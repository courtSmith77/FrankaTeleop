"""
Collect data for training model.

"""
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from std_srvs.srv import Empty

import cv2 as cv
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import csv

class DataCollection(Node):

    def __init__(self):
        super().__init__('data_collection')

        # create callback groups
        self.desired_callback_group = MutuallyExclusiveCallbackGroup()

        # create subscribers
        self.desired_ee_subscriber = self.create_subscription(Pose, '/desired_ee_pose', self.desired_ee_callback, 10, callback_group=self.desired_callback_group)
        self.end_effector_raw_sub = self.create_subscription(Image, '/d405/color/image_rect_raw', self.end_effector_image_callback, 10)
        self.scene_image_raw_sub = self.create_subscription(Image, '/d435/color/image_raw', self.scene_image_callback, 10)

        # create service
        self.record_srv = self.create_service(Empty, '/record', self.record_callback)

        # create timer
        self.timer = self.create_timer(1/30, self.timer_callback)

        self.bridge = CvBridge()

        self.received_ee_pose = False
        self.received_ee_image = False
        self.received_scene_image = False
        self.start_recording = False

        self.pos_data = './data/position.csv'

        self.count = 0
        self.end = True

    def desired_ee_callback(self, msg):
        """Callback for the desired ee pose callback"""
        self.desired_ee = msg
        self.received_ee_pose = True

    def end_effector_image_callback(self, msg):
        """Callback for the end effector image callback"""
        self.ee_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.received_ee_image = True

    def scene_image_callback(self, msg):
        """Callback for the scene image callback"""
        self.scene_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.received_scene_image = True

    def record_callback(self, request, response):
        """Callback for the strat recording callback"""
        self.start_recording = True
        self.get_logger().info('Starting to record...')
        return response

    def timer_callback(self):
        """Callback for the timer."""

        if self.start_recording:

            if self.received_ee_pose and self.received_ee_image and self.received_scene_image and self.end:

                ee_data = [self.desired_ee.position.x, self.desired_ee.position.y, self.desired_ee.position.z]
                self.get_logger().info(f'{ee_data}')
                with open(self.pos_data, mode='a') as csv_file:
                    csv_writer = csv.writer(csv_file)
                    csv_writer.writerow(ee_data)

                ee_img_name = f'./data/ee_img_{self.count}.jpg'
                scene_img_name = f'./data/scene_img_{self.count}.jpg'

                cv.imwrite(ee_img_name, self.ee_image)
                cv.imwrite(scene_img_name, self.scene_image)

                self.count+=1
                self.received_ee_pose = False
                self.received_ee_image = False
                self.received_scene_image = False

                if self.count % 10 == 0:
                    self.get_logger().info(f'Received {self.count} messages')

def main(args=None):
    rclpy.init(args=args)

    data_collection = DataCollection()

    rclpy.spin(data_collection)


if __name__ == '__main__':
    main()





















