from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_fake_hardware", default_value="true",
                                  description="whether or not to use fake hardware."),
            DeclareLaunchArgument(name="use_rviz", default_value="true",
                                  description="whether or not to use rviz."),
            DeclareLaunchArgument(name="collect_data", default_value="false",
                                  description="whether or not to use rviz."),
            DeclareLaunchArgument(name="robot_ip", default_value="dont-care",
                                  description="IP address of the robot"),
            DeclareLaunchArgument(name="use_realsense", default_value="true",
                                  description="whether or not to use realsense camera."),
            DeclareLaunchArgument(name="run_franka_teleop", default_value="true",
                                  description="whether or not to run franka teleop."),
            DeclareLaunchArgument(name="rviz_file", default_value="integrate_servo.rviz",
                                  description="rviz file to use."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution(
                    [FindPackageShare('franka_teleop'), 'launch', 'franka_rviz.launch.py'])]),
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                launch_arguments={'robot_ip': LaunchConfiguration("robot_ip"),
                                  'use_fake_hardware': LaunchConfiguration("use_fake_hardware"),
                                  'use_rviz': 'true',
                                  'rviz_file': PathJoinSubstitution([FindPackageShare('cv_franka_bridge'),'config',LaunchConfiguration('rviz_file')])}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution(
                    [FindPackageShare('franka_teleop'), 'launch', 'franka_servo.launch.py'])]),
                condition=IfCondition(LaunchConfiguration("run_franka_teleop")),
                launch_arguments={'robot_ip': LaunchConfiguration("robot_ip"),
                                  'use_fake_hardware': LaunchConfiguration("use_fake_hardware"),
                                  'use_rviz': 'false'}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution(
                    [FindPackageShare('commandmode'), 'launch', 'camera.launch.py'])]),
                launch_arguments={'use_realsense': LaunchConfiguration("use_realsense")}.items(),
                condition=IfCondition(LaunchConfiguration("use_realsense"))
            ),
            Node(
                package="cv_franka_bridge",
                executable="cv_franka_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_realsense")),
            ),
            Node(
                package="cv_franka_bridge",
                executable="data_collection",
                output="screen",
                condition=IfCondition(LaunchConfiguration("collect_data")),
            ),
        ]
    )
