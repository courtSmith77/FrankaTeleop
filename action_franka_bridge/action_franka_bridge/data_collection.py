"""
Collect data for training model.

This node reads in image and position data during demonstrations and
saves the data to train the models.

PUBLISHERS:
  + /desired_ee_pose (Pose) - The desired end effector position
  + /d405/color/image_rect_raw (Image) - The end effector camera raw image feed
  + /d435/color/image_raw (Image) - The scene camera raw image feed
SERVICES:
  + /record (Empty) - Enables saving the image and scene data
"""
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from std_srvs.srv import Empty

import cv2 as cv
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

import csv
import numpy as np
from datetime import datetime

class DataCollection(Node):

    def __init__(self):
        super().__init__('data_collection')

        # subscriptions for actions
        self.predicted_action_sub = self.create_subscription(Float32MultiArray, 'predicted_action', self.predicted_action_callback, 10)
        self.action_horizon_sub = self.create_subscription(Float32MultiArray, 'action_horizon', self.action_horizon_callback, 10)
        self.current_action_sub = self.create_subscription(Float32MultiArray, 'current_action', self.current_action_callback, 10)
        self.ee_at_action_sub = self.create_subscription(Float32MultiArray, 'ee_before_action', self.ee_before_action_callback, 10)
        self.ee_all_time_sub = self.create_subscription(Float32MultiArray, 'ee_all_time', self.ee_all_time_callback, 10)

        # create service
        self.record_srv = self.create_service(Empty, '/record', self.record_callback)

        self.bridge = CvBridge()

        self.received_ee_pose = False
        self.received_ee_image = False
        self.received_scene_image = False
        self.start_recording = False

        self.count = 0
        self.end = True

    def predicted_action_callback(self, msg):
        """Callback for saving the predicted action sequences."""
        if self.start_recording:
            pa_arr = msg.data
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size

            with open('./data/predicted_action.csv', mode='a') as self.pa_file:
                self.pa_csv_writer = csv.writer(self.pa_file)
                pa = np.array(pa_arr).reshape((rows,cols))
                curr_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                time_column = np.full((pa.shape[0], 1), curr_time)
                pa_with_time = np.hstack((time_column, pa))
                self.pa_csv_writer.writerows(pa_with_time)

    def action_horizon_callback(self, msg):
        """Callback for saving the entire action horizon sequences."""
        if self.start_recording:
            ah_arr = msg.data
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size

            with open('./data/action_horizon.csv', mode='a') as self.ah_file:
                self.ah_csv_writer = csv.writer(self.ah_file)
                ah = np.array(ah_arr).reshape((rows,cols))
                curr_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                time_column = np.full((ah.shape[0], 1), curr_time)
                ah_with_time = np.hstack((time_column, ah))
                self.ah_csv_writer.writerows(ah_with_time)
    
    def current_action_callback(self, msg):
        """Callback for saving the current action about to be executed."""
        if self.start_recording:
            ca_arr = list(msg.data)

            with open('./data/current_action.csv', mode='a') as self.ca_file:
                self.ca_csv_writer = csv.writer(self.ca_file)
                curr_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                ca_with_time = [curr_time] + ca_arr
                self.ca_csv_writer.writerow(ca_with_time)

    def ee_before_action_callback(self, msg):
        """Callback for saving the ee pose before executing the current action."""
        if self.start_recording:
            eb_arr = list(msg.data)

            with open('./data/ee_before_action.csv', mode='a') as self.eb_file:
                self.eb_csv_writer = csv.writer(self.eb_file)
                curr_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                eb_with_time = [curr_time] + eb_arr
                self.eb_csv_writer.writerow(eb_with_time)

    def ee_all_time_callback(self, msg):
        """Callback for saving the ee position at all times."""
        if self.start_recording:
            eat_arr = list(msg.data)

            with open('./data/ee_all_time.csv', mode='a') as self.eat_file:
                self.eat_csv_writer = csv.writer(self.eat_file)
                curr_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                eat_with_time = [curr_time] + eat_arr
                self.eat_csv_writer.writerow(eat_with_time)

    def record_callback(self, request, response):
        """Callback for the start recording callback"""
        self.start_recording = True
        self.get_logger().info('Starting to record...')
        return response

def main(args=None):
    rclpy.init(args=args)
    data_collection = DataCollection()
    rclpy.spin(data_collection)

if __name__ == '__main__':
    main()





















