"""Tests for launch-time CPU affinity configuration."""

import importlib.util
from pathlib import Path

import pytest
import yaml


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "air_ground_servo.launch.py"
CONFIG_FILE = Path(__file__).parents[1] / "config" / "air_ground_servo.yaml"
SPEC = importlib.util.spec_from_file_location("air_ground_servo_launch", LAUNCH_FILE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_taskset_prefix_pins_process_before_node_start():
    assert MODULE._taskset_prefix("6,7") == "taskset -c 6,7"


def test_taskset_prefix_allows_disabling_affinity():
    assert MODULE._taskset_prefix("  ") is None


@pytest.mark.parametrize("cpu_set", ["6;7", "cpu6,7", "6 7", "6,,7"])
def test_taskset_prefix_rejects_invalid_cpu_list(cpu_set):
    with pytest.raises(RuntimeError, match="CPU list"):
        MODULE._taskset_prefix(cpu_set)


def test_realsense_config_targets_fixed_d435i_serial():
    realsense, _ = MODULE._load_config(str(CONFIG_FILE))

    assert realsense["serial_no"] == "_327122074056"


def test_servo_config_defines_target_point_topic():
    _, servo = MODULE._load_config(str(CONFIG_FILE))

    assert servo["point_topic"] == "/air_ground_servo/target_point"


def test_servo_config_defines_target_geometry_parameters():
    _, servo = MODULE._load_config(str(CONFIG_FILE))

    assert servo["outer_diameter_m"] == 0.50
    assert servo["inner_diameter_m"] == 0.30
    assert servo["canny_low_threshold"] < servo["canny_high_threshold"]


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
