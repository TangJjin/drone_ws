import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("drone_perception")
    config = os.path.join(package_share, "config", "line_vision.yaml")
    usb_cam_launch = os.path.join(
        get_package_share_directory("hobot_usb_cam"), "launch", "hobot_usb_cam.launch.py")
    codec_launch = os.path.join(
        get_package_share_directory("hobot_codec"), "launch", "hobot_codec_decode.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config),
        DeclareLaunchArgument(
            "usb_video_device",
            default_value=(
                "/dev/v4l/by-id/usb-12MP_U3_Camera_12MP_U3_Camera_2601230002-video-index0"),
        ),
        DeclareLaunchArgument("usb_image_width", default_value="1280"),
        DeclareLaunchArgument("usb_image_height", default_value="720"),
        DeclareLaunchArgument("usb_framerate", default_value="60"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(usb_cam_launch),
            launch_arguments={
                "usb_video_device": LaunchConfiguration("usb_video_device"),
                "usb_image_width": LaunchConfiguration("usb_image_width"),
                "usb_image_height": LaunchConfiguration("usb_image_height"),
                "usb_framerate": LaunchConfiguration("usb_framerate"),
                "usb_pixel_format": "mjpeg",
                "usb_io_method": "mmap",
                "usb_zero_copy": "True",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(codec_launch),
            launch_arguments={
                "codec_in_mode": "shared_mem",
                "codec_in_format": "jpeg",
                "codec_out_mode": "shared_mem",
                "codec_out_format": "nv12",
                "codec_sub_topic": "/hbmem_img",
                "codec_pub_topic": "/line_vision/nv12",
                "codec_input_framerate": LaunchConfiguration("usb_framerate"),
            }.items(),
        ),
        Node(
            package="drone_perception",
            executable="line_vision_node",
            name="line_vision_node",
            output="screen",
            emulate_tty=True,
            arguments=["--ros-args", "--log-level", "warn"],
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
        ),
    ])
