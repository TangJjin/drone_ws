from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    color_topic = LaunchConfiguration("color_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    debug_view = LaunchConfiguration("debug_view")
    enable_bpu = LaunchConfiguration("enable_bpu")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "color_topic",
                default_value="/camera/rgb/image_raw",
                description="Gazebo RGB topic for qr_vision_node.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/rgb/camera_info",
                description="Gazebo camera info topic for qr_vision_node.",
            ),
            DeclareLaunchArgument(
                "debug_view",
                default_value="true",
                description="Whether to open the debug preview window.",
            ),
            DeclareLaunchArgument(
                "enable_bpu",
                default_value="false",
                description="Disable hardware BPU for local Gazebo tests by default.",
            ),
            Node(
                package="drone_perception",
                executable="qr_vision_node",
                name="qr_vision_node",
                output="screen",
                parameters=[
                    {
                        "color_topic": color_topic,
                        "camera_info_topic": camera_info_topic,
                        "debug_view": debug_view,
                        "enable_bpu": enable_bpu,
                    }
                ],
            ),
        ]
    )
