from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (DeclareLaunchArgument, Shutdown, IncludeLaunchDescription,
                            SetLaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration, EqualsSubstitution,
                                  Command, FindExecutable, PythonExpression)
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python import get_package_share_directory
import yaml
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_realsense", default_value="true",
            description="Use the Realsense Camera. If 'false', will attempt to use usb camera or built in webcam"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ),
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration("use_realsense"), "true")),
            launch_arguments={
                "camera_name": "d405",
                "device_type": "d405",
                "color_height": "240",
                "color_width": "320",
                "color_fps": "30",
                "depth_height": "240",
                "depth_width": "320",
                "depth_fps": "30",
                "align_depth.enable": "true",
                "pointcloud.enable": "true",
                "colorizer.enable": "true",
                "decimation_filter.enable": "true",
                "spatial_filter.enable": "true",
                "temporal_filter.enable": "true",
                "disparity_filter.enable": "true",
                "hole_filling_filter.enable": "true",
                "hdr_merge.enable": "true",
                "json_file_path": get_package_share_directory("handcv") + "/config/high_density_preset.json",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ),
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration("use_realsense"), "true")),
            launch_arguments={
                "camera_name": "d435",
                "device_type": "d435",
                "color_height": "240",
                "color_width": "320",
                "color_fps": "30",
                "depth_height": "240",
                "depth_width": "320",
                "depth_fps": "30",
                "align_depth.enable": "true",
                "pointcloud.enable": "true",
                "colorizer.enable": "true",
                "decimation_filter.enable": "true",
                "spatial_filter.enable": "true",
                "temporal_filter.enable": "true",
                "disparity_filter.enable": "true",
                "hole_filling_filter.enable": "true",
                "hdr_merge.enable": "true",
                "json_file_path": get_package_share_directory("handcv") + "/config/high_density_preset.json",
            }.items(),
        ),
        Node(
            package="handcv",
            executable="handcv",
            output='screen',
            prefix="xterm -e",
        ),
    ])
