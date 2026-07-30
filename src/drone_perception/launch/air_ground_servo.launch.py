"""Launch the D435i RGBD driver and the air-ground servo debug node."""

import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


REALSENSE_TYPES = {
    "camera_namespace": str,
    "camera_name": str,
    "enable_color": bool,
    "rgb_camera.color_profile": str,
    "enable_depth": bool,
    "depth_module.depth_profile": str,
    "enable_sync": bool,
    "enable_rgbd": bool,
    "align_depth.enable": bool,
}

SERVO_TYPES = {
    "rgbd_topic": str,
    "expected_width": int,
    "expected_height": int,
    "expected_fps": (int, float),
    "center_x": int,
    "center_y": int,
    "sample_window_size": int,
    "min_depth_m": (int, float),
    "max_depth_m": (int, float),
    "fps_smoothing_alpha": (int, float),
    "window_name": str,
    "debug_view": bool,
    "log_throttle_ms": int,
}

SERVO_FLOAT_PARAMETERS = {
    "expected_fps",
    "min_depth_m",
    "max_depth_m",
    "fps_smoothing_alpha",
}


def _require_mapping(parent, key):
    value = parent.get(key)
    if not isinstance(value, dict):
        raise RuntimeError(f"'{key}' must be a YAML mapping")
    return value


def _validate_section(section_name, values, expected_types):
    missing = sorted(set(expected_types) - set(values))
    unknown = sorted(set(values) - set(expected_types))
    if missing:
        raise RuntimeError(f"'{section_name}' is missing parameters: {', '.join(missing)}")
    if unknown:
        raise RuntimeError(f"'{section_name}' has unsupported parameters: {', '.join(unknown)}")

    for name, expected_type in expected_types.items():
        value = values[name]
        if isinstance(value, bool) and expected_type != bool:
            raise RuntimeError(f"'{section_name}.{name}' has invalid boolean value")
        if not isinstance(value, expected_type):
            raise RuntimeError(
                f"'{section_name}.{name}' has invalid type {type(value).__name__}"
            )


def _as_launch_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _taskset_prefix(cpu_set):
    normalized = cpu_set.strip()
    if normalized and not re.fullmatch(r"[0-9]+(?:[,-][0-9]+)*", normalized):
        raise RuntimeError("cpu_set must be a CPU list like '6,7' or '4-7'")
    return f"taskset -c {normalized}" if normalized else None


def _load_config(path):
    if not os.path.isfile(path):
        raise RuntimeError(f"air-ground servo config file does not exist: {path}")
    with open(path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise RuntimeError("air-ground servo config root must be a YAML mapping")

    unknown_sections = sorted(set(config) - {"realsense", "air_ground_servo_node"})
    if unknown_sections:
        raise RuntimeError(f"unsupported config sections: {', '.join(unknown_sections)}")

    realsense = _require_mapping(config, "realsense")
    servo = _require_mapping(config, "air_ground_servo_node")
    servo_parameters = _require_mapping(servo, "ros__parameters")
    if set(servo) != {"ros__parameters"}:
        raise RuntimeError("'air_ground_servo_node' only supports the 'ros__parameters' key")

    _validate_section("realsense", realsense, REALSENSE_TYPES)
    _validate_section("air_ground_servo_node.ros__parameters", servo_parameters, SERVO_TYPES)

    # ROS 2参数类型在启动后不可由整数隐式覆盖为double，先统一数值类型。
    servo_parameters = dict(servo_parameters)
    for name in SERVO_FLOAT_PARAMETERS:
        servo_parameters[name] = float(servo_parameters[name])

    profile_pattern = re.compile(r"^[1-9][0-9]*x[1-9][0-9]*x[1-9][0-9]*$")
    for profile_name in ("rgb_camera.color_profile", "depth_module.depth_profile"):
        if not profile_pattern.fullmatch(realsense[profile_name]):
            raise RuntimeError(f"'realsense.{profile_name}' must use WIDTHxHEIGHTxFPS")

    return realsense, servo_parameters


def _launch_setup(context):
    config_path = os.path.abspath(LaunchConfiguration("config_file").perform(context))
    realsense, servo_parameters = _load_config(config_path)
    servo_prefix = _taskset_prefix(LaunchConfiguration("cpu_set").perform(context))

    realsense_launch = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )
    realsense_arguments = {
        name: _as_launch_value(value) for name, value in realsense.items()
    }
    # rs_launch.py用字面量"''"表示不加载驱动YAML，避免继承本launch的双分区配置。
    realsense_arguments["config_file"] = "''"

    return [
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(realsense_launch),
                ),
            ],
            scoped=True,
            forwarding=False,
            launch_configurations=realsense_arguments,
        ),
        Node(
            package="drone_perception",
            executable="air_ground_servo_node",
            name="air_ground_servo_node",
            output="screen",
            emulate_tty=True,
            prefix=servo_prefix,
            parameters=[servo_parameters],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("drone_perception")
    default_config = os.path.join(package_share, "config", "air_ground_servo.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to the commented air-ground RGBD YAML configuration",
        ),
        DeclareLaunchArgument(
            "cpu_set",
            default_value="6,7",
            description="CPU list for taskset before the RGBD node starts; empty disables pinning",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
