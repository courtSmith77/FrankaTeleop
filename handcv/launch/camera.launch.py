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
        Node(
            package="handcv",
            executable="handcv",
            output='screen',
            prefix="xterm -e",
        ),
    ])
