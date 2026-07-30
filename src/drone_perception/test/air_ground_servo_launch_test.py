"""Tests for launch-time CPU affinity configuration."""

import importlib.util
from pathlib import Path

import pytest


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
