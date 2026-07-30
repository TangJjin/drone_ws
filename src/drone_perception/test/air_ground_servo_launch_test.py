"""Tests for launch-time CPU affinity configuration."""

import importlib.util
from pathlib import Path

import pytest
import yaml


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "air_ground_servo.launch.py"
CONFIG_FILE = Path(__file__).parents[1] / "config" / "air_ground_servo.yaml"
NODE_FILE = Path(__file__).parents[1] / "src" / "air_ground_servo_node.cpp"
SPEC = importlib.util.spec_from_file_location("air_ground_servo_launch", LAUNCH_FILE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_taskset_prefix_pins_process_before_node_start():
    assert MODULE._taskset_prefix("6,7") == "taskset -c 6,7"


def test_servo_launch_does_not_include_the_realsense_driver():
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert "realsense2_camera" not in launch_source
    assert "IncludeLaunchDescription" not in launch_source


def test_d435i_driver_uses_the_official_realsense_launch_directly():
    d435i_launch_file = Path(__file__).parents[1] / "launch" / "d435i_rgbd.launch.py"

    assert not d435i_launch_file.exists()


def test_taskset_prefix_allows_disabling_affinity():
    assert MODULE._taskset_prefix("  ") is None


@pytest.mark.parametrize("cpu_set", ["6;7", "cpu6,7", "6 7", "6,,7"])
def test_taskset_prefix_rejects_invalid_cpu_list(cpu_set):
    with pytest.raises(RuntimeError, match="CPU list"):
        MODULE._taskset_prefix(cpu_set)


def test_realsense_config_targets_fixed_d435i_serial():
    realsense, _ = MODULE._load_config(str(CONFIG_FILE))

    assert realsense["serial_no"] == "_327122074056"


def test_servo_config_defines_control_target_topic():
    _, servo = MODULE._load_config(str(CONFIG_FILE))

    assert servo["servo_topic"] == "/vision/servo/target"


def test_servo_node_publishes_single_frame_confirmed_vision_target():
    node_source = NODE_FILE.read_text(encoding="utf-8")

    assert "drone_msgs/msg/vision_servo_target.hpp" in node_source
    assert "drone_msgs::msg::VisionServoTarget" in node_source
    assert "output.confirmed = valid;" in node_source


def test_servo_config_defines_ellipse_marker_parameters():
    _, servo = MODULE._load_config(str(CONFIG_FILE))

    assert servo["min_ring_radius_px"] < servo["max_ring_radius_px"]
    assert servo["min_circularity"] == 0.2
    assert servo["min_axis_ratio"] == 0.4
    assert servo["min_inner_ring_score"] == 0.2
    assert servo["min_cross_score"] == 0.3
    assert servo["gaussian_blur_kernel"] % 2 == 1


def test_servo_config_uses_binary_debug_view():
    _, servo = MODULE._load_config(str(CONFIG_FILE))

    assert servo["view_mode"] == "BINARY"


def test_servo_config_rejects_unknown_view_mode(tmp_path):
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    config["air_ground_servo_node"]["ros__parameters"]["view_mode"] = "EDGE"
    invalid_config = tmp_path / "invalid_view_mode.yaml"
    invalid_config.write_text(yaml.safe_dump(config), encoding="utf-8")

    with pytest.raises(RuntimeError, match="view_mode.*must be RGB, GRAY or BINARY"):
        MODULE._load_config(str(invalid_config))


def test_realsense_config_uses_supported_50_hz_power_line_frequency():
    realsense, _ = MODULE._load_config(str(CONFIG_FILE))

    assert realsense["rgb_camera.power_line_frequency"] == 1


def test_realsense_config_rejects_unsupported_power_line_frequency(tmp_path):
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    config["realsense"]["rgb_camera.power_line_frequency"] = 3
    invalid_config = tmp_path / "invalid_power_line_frequency.yaml"
    invalid_config.write_text(yaml.safe_dump(config), encoding="utf-8")

    with pytest.raises(RuntimeError, match="must be 0, 1, or 2"):
        MODULE._load_config(str(invalid_config))
