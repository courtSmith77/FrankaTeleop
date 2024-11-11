"""
Interpret command mode from user, and converts actions from the diffusion
model into robot motion.

This node interprets three command states from the user: Begin ('b'), Action
('a'), Pause ('p' or 's'). These states are used to control whether or not the
robot performs the actions from the subscribed /predicted_action topic. Actions
and current robot pose are used within two PD loops, one for position and one for
orientation, to control the robot.

SUBSCRIBERS:
  + /predicted_action (Pose) - The next action position from the diffusion model.
  + /command_mode (String) - The command mode based on the key pressed.
PUBLISHERS:
  + /text_marker (Marker) - The text marker that is published to RViz.
  + /bounding_box (Marker) - The bounding box marker that is published to RViz.
  + /desired_ee_pose (Pose) - The desired pose of the end effector.
SERVICE CLIENTS:
  + /robot_waypoints (PlanPath) - The service that plans and executes the robot's
    motion.
  + /record (Empty) - The service that initiates recording the demonstration data.
"""
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from franka_teleop.srv import PlanPath

from visualization_msgs.msg import Marker

from std_srvs.srv import Empty
from std_msgs.msg import String, Float32MultiArray

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import PoseStamped
import tf2_ros
from tf_transformations import quaternion_from_euler, euler_from_quaternion

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor

import numpy as np
import csv

class ActionFrankaBridge(Node):

    def __init__(self):
        super().__init__('action_franka_bridge')

        # frequency parameter
        self.declare_parameter('frequency', 10.0, ParameterDescriptor(description='Frequency (hz) of the timer callback'))
        self.timer_freqency = self.get_parameter('frequency').get_parameter_value().double_value

        # create callback groups
        self.waypoint_callback_group = MutuallyExclusiveCallbackGroup()
        self.command_mode_callback_group = MutuallyExclusiveCallbackGroup()

        # create subscribers
        self.action_subscriber = self.create_subscription(Float32MultiArray, 'predicted_action', self.action_callback, 10, callback_group=self.waypoint_callback_group)
        self.command_mode_subscriber = self.create_subscription(String, 'command_mode', self.command_mode_callback, 10, callback_group=self.command_mode_callback_group)

        # create publishers
        self.text_marker_publisher = self.create_publisher(Marker, 'text_marker', 10)
        self.bounding_box_publisher = self.create_publisher(Marker, 'bounding_box', 10)

        # create clients
        self.waypoint_client = self.create_client(PlanPath, 'robot_waypoints')
        self.waypoint_client.wait_for_service(timeout_sec=2.0)
        self.record_client = self.create_client(Empty, 'record')
        self.record_client.wait_for_service(timeout_sec=2.0)

        # create timer
        self.timer = self.create_timer((1.0/self.timer_freqency), self.timer_callback)

        # create tf buffer and listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # create class variables
        self.text_marker = self.create_text_marker("Press_'b'_to_begin_inference")

        self.initial_ee_pose = Pose(position=Point(x=0.20, y=0.402, z=0.085),
                                    orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0))
        self.desired_ee_pose = self.initial_ee_pose
        self.move_robot = False
        self.prev_gesture = None

        # action variables
        self.pending_action = False
        self.action_array = None
        self.action_counter = 0

        self.lower_distance_threshold = 0.0
        self.upper_distance_threshold = 0.05

        # PID parameters
        self.kp_angle = 1.0
        self.ki_angle = 0.0
        self.kd_angle = 0.01
        self.roll_error_prior = 0
        self.pitch_error_prior = 0
        self.yaw_error_prior = 0

        self.x_limits = [0.15, 1.0]
        self.y_limits = [-0.75, 0.6]
        self.y_inner = [-0.15, 0.15]
        self.z_limits = [0.07, 0.75]
        self.bounding_box_marker = self.create_box_marker()

        self.count = 0

    def create_text_marker(self, text):
        """Create a text marker."""
        marker = Marker()
        marker.header.frame_id = "panda_link0"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.text = text
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        return marker

    def create_box_marker(self):
        """Create a line strip that represents the bounding box."""
        marker = Marker()
        marker.header.frame_id = "panda_link0"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.points = [
                Point(x=self.x_limits[0], y=self.y_limits[0], z=self.z_limits[0]),
                Point(x=self.x_limits[1], y=self.y_limits[0], z=self.z_limits[0]),
                Point(x=self.x_limits[1], y=self.y_limits[1], z=self.z_limits[0]),
                Point(x=self.x_limits[0], y=self.y_limits[1], z=self.z_limits[0]),
                Point(x=self.x_limits[0], y=self.y_limits[0], z=self.z_limits[0]),
                Point(x=self.x_limits[0], y=self.y_limits[0], z=self.z_limits[1]),
                Point(x=self.x_limits[1], y=self.y_limits[0], z=self.z_limits[1]),
                Point(x=self.x_limits[1], y=self.y_limits[1], z=self.z_limits[1]),
                Point(x=self.x_limits[0], y=self.y_limits[1], z=self.z_limits[1]),
                Point(x=self.x_limits[0], y=self.y_limits[0], z=self.z_limits[1]),
                Point(x=self.x_limits[1], y=self.y_limits[0], z=self.z_limits[1]),
                Point(x=self.x_limits[1], y=self.y_limits[0], z=self.z_limits[0]),
                Point(x=self.x_limits[1], y=self.y_limits[1], z=self.z_limits[0]),
                Point(x=self.x_limits[1], y=self.y_limits[1], z=self.z_limits[1]),
                Point(x=self.x_limits[0], y=self.y_limits[1], z=self.z_limits[1]),
                Point(x=self.x_limits[0], y=self.y_limits[1], z=self.z_limits[0])
                ]
        marker.scale.x = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        return marker

    def get_transform(self, target_frame, source_frame):
        """Get the transform between two frames."""
        try:
            trans = self.buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            translation = trans.transform.translation
            rotation = trans.transform.rotation
            return translation, rotation

        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f"Lookup exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f"Connectivity exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f"Extrapolation exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]

    def get_ee_pose(self):
        """Get the current pose of the end-effector."""

        try:
            ee_home_pos, ee_home_rot = self.get_transform("panda_link0", "panda_hand_tcp")
            ee_pose = Pose()
            ee_pose.position.x = ee_home_pos.x
            ee_pose.position.y = ee_home_pos.y
            ee_pose.position.z = ee_home_pos.z
            ee_pose.orientation.x = ee_home_rot.x
            ee_pose.orientation.y = ee_home_rot.y
            ee_pose.orientation.z = ee_home_rot.z
            ee_pose.orientation.w = ee_home_rot.w
            return ee_pose
        except:
            self.get_logger().info('Replacing EE pose with Desired Pose')
            return self.desired_ee_pose

    def angle_correction(self, current_euler, desired_euler):
        """Calculate the angle corrections."""

        # Orientation PID loops
        if current_euler[0] < 0:
            current_euler[0] += 2 * np.pi
        if desired_euler[0] < 0:
            desired_euler[0] += 2 * np.pi
        roll_error = desired_euler[0] - current_euler[0]
        pitch_error = desired_euler[1] - current_euler[1]
        yaw_error = desired_euler[2] - current_euler[2]

        roll_derivative = (roll_error - self.roll_error_prior)
        pitch_derivative = (pitch_error - self.pitch_error_prior)
        yaw_derivative = (yaw_error - self.yaw_error_prior)

        roll_output = self.kp_angle * roll_error - self.kd_angle * roll_derivative
        pitch_output = self.kp_angle * pitch_error + self.kd_angle * pitch_derivative
        yaw_output = self.kp_angle * yaw_error + self.kd_angle * yaw_derivative

        euler_output = [roll_output, -pitch_output, -yaw_output]

        self.roll_error_prior = roll_error
        self.pitch_error_prior = pitch_error
        self.yaw_error_prior = yaw_error

        return euler_output

    def check_boundaries(self):
        """Check for boundary limits."""

        if (self.desired_ee_pose.position.x < self.x_limits[0] or self.desired_ee_pose.position.x > self.x_limits[1]):
            self.desired_ee_pose.position.x = self.x_limits[0] if self.desired_ee_pose.position.x < self.x_limits[0] else self.x_limits[1]
            self.get_logger().info('Trying to go to far out of X')
        if (self.desired_ee_pose.position.y < self.y_limits[0] or self.desired_ee_pose.position.y > self.y_limits[1]):
            self.desired_ee_pose.position.y = self.y_limits[0] if self.desired_ee_pose.position.y < self.y_limits[0] else self.y_limits[1]
            self.get_logger().info('Trying to go to far out of Y')
        if ((self.desired_ee_pose.position.y < self.y_inner[1] and self.desired_ee_pose.position.y > self.y_inner[0]) and self.desired_ee_pose.position.x < self.x_limits[0]):
            self.get_logger().info('Too close to base!!!!!!!!!!!!!')
            upper_diff = abs(self.y_inner[1] - self.desired_ee_pose.position.y)
            lower_diff = abs(self.y_inner[0] - self.desired_ee_pose.position.y)
            if upper_diff < lower_diff:
                self.desired_ee_pose.position.y = self.y_inner[1]
            else:
                self.desired_ee_pose.position.y = self.y_inner[0]
        if (self.desired_ee_pose.position.z < self.z_limits[0] or self.desired_ee_pose.position.z > self.z_limits[1]):
            self.desired_ee_pose.position.z = self.z_limits[0] if self.desired_ee_pose.position.z < self.z_limits[0] else self.z_limits[1]

    def action_callback(self, msg):
        """Callback for the action subscriber."""

        # only save action once previous action set is done executing
        if not self.pending_action:
            arr = msg.data
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size

            self.action_array = np.array(arr).reshape((rows,cols))
            # with open('./actions_executed.csv', mode='a') as csv_file:
            #     csv_writer = csv.writer(csv_file)
            #     csv_writer.writerows(self.action_array)
            self.pending_action = True

            # self.get_logger().info(f'Original Message Arr = {arr}')
            # self.get_logger().info(f'Number of actions received = {self.action_array.shape[0]}')
            self.get_logger().info(f'Action pairs = {self.action_array}')

    def command_mode_callback(self, msg):
        """Callback for the command mode subscriber."""
        if msg.data == "Begin" or msg.data == "Pause":
            self.desired_ee_pose = self.get_ee_pose()
            self.text_marker = self.create_text_marker(msg.data)
            self.move_robot = False

        if msg.data == "Pause":
            # make sure robot does not move if the command mode is 'Pause'
            self.move_robot = False

        if self.prev_gesture == "Begin" and msg.data == "Action":
            self.get_logger().info('Allowing robot to move now, if actions are sent')
            self.move_robot = True

            self.desired_ee_pose = self.get_ee_pose()
            quat = quaternion_from_euler(-np.pi, 0.0, 0.0)
            self.desired_ee_pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        self.prev_gesture = msg.data

    async def timer_callback(self):
        """Callback for the timer."""
        # publish a text marker with the current gesture
        self.text_marker_publisher.publish(self.text_marker)
        self.bounding_box_publisher.publish(self.bounding_box_marker)

        if self.move_robot:
            # self.get_logger().info('Can Move Robot!!!')
                
            if self.action_array is not None and self.action_counter < self.action_array.shape[0] and self.pending_action:
                # self.get_logger().info(f'Pulling from action array = {self.action_array}')

                desired_x = float(self.action_array[self.action_counter][0])
                desired_y = float(self.action_array[self.action_counter][1])

                current_pos = self.get_ee_pose()
                diff_vector = np.array([desired_x, desired_y]) - np.array([current_pos.position.x, current_pos.position.y])
                distance = np.linalg.norm(diff_vector)
                
                if distance < self.lower_distance_threshold:
                    self.get_logger().info("Trying to move too close. Staying still.")
                    self.desired_ee_pose = self.get_ee_pose()
                elif distance > self.upper_distance_threshold:
                    self.get_logger().info("Trying to move too far, clipping.")

                    coeff = self.upper_distance_threshold/distance
                    converted_vector = coeff*diff_vector + np.array([current_pos.position.x, current_pos.position.y])
                    self.desired_ee_pose.position.x = converted_vector[0]
                    self.desired_ee_pose.position.y = converted_vector[1]
                    
                else:
                    self.desired_ee_pose.position.x = desired_x
                    self.desired_ee_pose.position.y = desired_y
                
                self.action_counter +=1

                self.get_logger().info(f'From Action, desired: x={self.desired_ee_pose.position.x}, y={self.desired_ee_pose.position.y}')

            else:
                self.pending_action = False
                self.action_counter = 0
                self.desired_ee_pose = self.get_ee_pose()
                self.get_logger().info('Only using EE for desired (robot move = true)')

        else:

            self.desired_ee_pose = self.get_ee_pose()
            self.get_logger().info('Only using EE for desired (robot move = false)')


        # Crop to bound area
        self.check_boundaries()

        try:
            ee_pose = self.get_ee_pose()
        except AttributeError as e:
            return

        # Use PID to correct angles
        current_angles = list(euler_from_quaternion([ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w]))
        desired_angles = list(euler_from_quaternion([1.0, 0.0, 0.0, 0.0]))
        euler_output = self.angle_correction(current_angles, desired_angles)

        self.get_logger().info(f'Desired_pos: x={self.desired_ee_pose.position.x} y={self.desired_ee_pose.position.y} z={0.085}')
        self.get_logger().info(f'EE_pos:      x={ee_pose.position.x} y={ee_pose.position.y} z={ee_pose.position.z}')

        # Publish Requested Path
        robot_move = PoseStamped()
        robot_move.header.frame_id = "panda_link0"
        robot_move.header.stamp = self.get_clock().now().to_msg()
        robot_move.pose.position.x = np.round((self.desired_ee_pose.position.x - ee_pose.position.x),4)
        robot_move.pose.position.y = np.round(-(self.desired_ee_pose.position.y - ee_pose.position.y),4)
        robot_move.pose.position.z = np.round(-(0.085 - ee_pose.position.z)/2.0,4)

        planpath_request = PlanPath.Request()
        planpath_request.waypoint = robot_move
        planpath_request.angles = euler_output
        future = self.waypoint_client.call_async(planpath_request)
        self.get_logger().info(f'Executing: x={robot_move.pose.position.x}, y={robot_move.pose.position.y}, z={robot_move.pose.position.z}')
        self.get_logger().info(f'Executing: roll={euler_output[0]}, pitch={euler_output[1]}, yaw={euler_output[2]}')

def main(args=None):
    rclpy.init(args=args)
    action_franka_bridge = ActionFrankaBridge()
    rclpy.spin(action_franka_bridge)

if __name__ == '__main__':
    main()






