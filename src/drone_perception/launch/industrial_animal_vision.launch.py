"""Launch the industrial-camera MPP and RKNN animal detector."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PROFILE_FILES = {
    "performance_230": "industrial_animal_performance_230.yaml",
    "vision_120": "industrial_animal_vision_120.yaml",
}

STRING_ARGUMENTS = ("camera_device", "model_path")
INTEGER_ARGUMENTS = (
    "camera_width", "camera_height", "camera_fps", "decode_width", "decode_height",
    "exposure_auto", "exposure_absolute", "exposure_auto_priority", "gain",
    "brightness", "contrast", "saturation", "gamma", "sharpness",
    "backlight_compensation", "white_balance_auto", "white_balance_temperature",
    "power_line_frequency", "focus_auto", "focus_absolute", "zoom_absolute",
)
FLOAT_ARGUMENTS = ("display_fps_limit", "confidence_threshold", "nms_threshold")
BOOLEAN_ARGUMENTS = ("display_enabled", "enable_zero_copy", "cpu_affinity_enabled")


def _launch_node(context):
    profile = LaunchConfiguration("camera_profile").perform(context)
    if profile not in PROFILE_FILES:
        choices = ", ".join(sorted(PROFILE_FILES))
        raise RuntimeError(f"Unknown camera_profile '{profile}'; choose: {choices}")

    package_share = get_package_share_directory("drone_perception")
    profile_path = os.path.join(package_share, "config", PROFILE_FILES[profile])
    default_model = os.path.join(
        package_share, "models", "animal_clean200_yolo11n_fp16.rknn"
    )
    overrides = {"camera_profile": profile}

    for name in STRING_ARGUMENTS:
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value
    overrides.setdefault("model_path", default_model)

    for name in INTEGER_ARGUMENTS:
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = int(value)
    for name in FLOAT_ARGUMENTS:
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = float(value)
    for name in BOOLEAN_ARGUMENTS:
        value = LaunchConfiguration(name).perform(context)
        if value:
            normalized = value.strip().lower()
            if normalized not in ("true", "false"):
                raise RuntimeError(f"{name} must be 'true' or 'false'")
            overrides[name] = normalized == "true"

    return [
        Node(
            package="drone_perception",
            executable="industrial_animal_vision_node",
            name="industrial_animal_vision",
            output="screen",
            emulate_tty=True,
            parameters=[profile_path, overrides],
        )
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "camera_profile",
            default_value="performance_230",
            description="performance_230 or vision_120",
        )
    ]
    for name in STRING_ARGUMENTS + INTEGER_ARGUMENTS + FLOAT_ARGUMENTS + BOOLEAN_ARGUMENTS:
        arguments.append(
            DeclareLaunchArgument(
                name,
                default_value="",
                description=f"Optional override for the '{name}' node parameter",
            )
        )
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_node)])
