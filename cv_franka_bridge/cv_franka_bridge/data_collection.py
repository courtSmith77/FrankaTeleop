"""
Collect data for training model.

"""
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray

import cv2 as cv
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import csv
import os
import datetime

class DataCollection(Node):

    def __init__(self):
        super().__init__('data_collection')

        # create callback groups
        self.desired_callback_group = MutuallyExclusiveCallbackGroup()

        # create subscribers
        # self.desired_ee_subscriber = self.create_subscription(Pose, '/desired_ee_pose', self.desired_ee_callback, 10, callback_group=self.desired_callback_group)
        self.desired_ee_subscriber = self.create_subscription(Float64MultiArray, '/desired_ee_pose', self.desired_ee_callback, 10, callback_group=self.desired_callback_group)

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

        self.count = 0

    def desired_ee_callback(self, msg):
        """Callback for the desired ee pose callback"""

        self.desired_ee_position = [msg.data[0], msg.data[1]]

        # self.desired_ee = msg
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

        if not self.start_recording:
            self.start_recording = True
            self.count = 0
            
            self.data_dir = datetime.datetime.now().strftime("%H:%M:%S")
            if not os.path.exists(self.data_dir):
                os.makedirs(self.data_dir)

            self.ee_img_dir = self.data_dir + '/ee_img/'
            self.scene_img_dir = self.data_dir + '/scene_img/'
            if not os.path.exists(self.ee_img_dir):
                os.makedirs(self.ee_img_dir)
            if not os.path.exists(self.scene_img_dir):
                os.makedirs(self.scene_img_dir)

            self.pos_file = self.data_dir + '/position.csv'
            with open(self.pos_file, mode='w') as csv_file:
                pass

            self.get_logger().info('Starting record...')
        else:
            self.start_recording = False
            self.get_logger().info('Stopping record...')

        return response

    def timer_callback(self):
        """Callback for the timer."""

        if self.start_recording:

            if self.received_ee_pose and self.received_ee_image and self.received_scene_image:

                # ee_data = [self.desired_ee.position.x, self.desired_ee.position.y, self.desired_ee.position.z]
                ee_data = self.desired_ee_position
                self.get_logger().info(f'{ee_data}')
                with open(self.pos_file, mode='a') as csv_file:
                    csv_writer = csv.writer(csv_file)
                    csv_writer.writerow(ee_data)

                ee_img_name = f'ee_img_{self.count}.jpg'
                scene_img_name = f'scene_img_{self.count}.jpg'

                cv.imwrite(self.ee_img_dir + ee_img_name, self.ee_image)
                cv.imwrite(self.scene_img_dir + scene_img_name, self.scene_image)

                self.count+=1
                self.received_ee_pose = False
                self.received_ee_image = False
                self.received_scene_image = False

                if self.count % 10 == 0:
                    self.get_logger().info(f'Received {self.count} messages')
            else:
                self.get_logger().info('Did not recieve enough data, not recording!!!!')

def main(args=None):
    rclpy.init(args=args)

    data_collection = DataCollection()

    rclpy.spin(data_collection)


if __name__ == '__main__':
    main()





















