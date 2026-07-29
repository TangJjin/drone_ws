import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("drone_line_vision")
    config = os.path.join(package_share, "config", "line_vision.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config),
        Node(
            package="drone_line_vision",
            executable="line_vision_node",
            name="line_vision_node",
            output="screen",
            emulate_tty=True,
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
        ),
    ])
