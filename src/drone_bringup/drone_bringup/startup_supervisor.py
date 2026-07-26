#!/usr/bin/env python3

import os
import sys
import time
import signal
import subprocess
from dataclasses import dataclass
from typing import List


@dataclass
class StartupStep:
    name: str
    command: List[str]
    ready_topic: str
    ready_type: str
    timeout_sec: int
    ready_qos_reliability: str = 'reliable'
    ready_stable_sec=2.0      # 话题至少连续稳定 2 秒
    ready_min_messages=5      # 至少收到 5 条消息
    ready_max_gap_sec=2     # 两条消息之间超过 2 秒就认为不稳定，重新计数
    post_ready_delay_sec=1.0  # ready 成功后再等 1 秒，再启动下一步


class StartupSupervisor:
    def __init__(self):
        self.processes = []
        self.running = True

        signal.signal(signal.SIGINT, self.handle_signal)
        signal.signal(signal.SIGTERM, self.handle_signal)

        wait_script = os.environ.get('WAIT_FOR_TOPIC_SCRIPT', 'YOUR_WAIT_SCRIPT_ABS_PATH_HERE')

        self.wait_script = wait_script
        self.steps = [
            StartupStep(
                name='mavros',
                command=['ros2', 'launch', 'mavros', 'px4.launch'],
                ready_topic='/mavros/state',
                ready_type='mavros_msgs/msg/State',
                timeout_sec=60,
                ready_qos_reliability='best_effort',
            ),
            StartupStep(
                name='livox_ros_driver2',
                command=['ros2', 'launch', 'livox_ros_driver2', 'msg_MID360_launch.py'],
                ready_topic='/livox/lidar',
                ready_type='livox_ros_driver2/msg/CustomMsg',
                timeout_sec=15,
                ready_qos_reliability='best_effort',
            ),
            StartupStep(
                name='fast_lio',
                command=['ros2', 'launch', 'fast_lio', 'mapping.launch.py'],
                ready_topic='/Odometry',
                ready_type='nav_msgs/msg/Odometry',
                timeout_sec=20,
                ready_qos_reliability='best_effort',
            ),
            StartupStep(
                name='fastlio_to_mavros',
                command=['ros2', 'run', 'drone_localization', 'fastlio_to_mavros_odom_out_node'],
                ready_topic='/mavros/odometry/out',
                ready_type='nav_msgs/msg/Odometry',
                timeout_sec=15,
                ready_qos_reliability='best_effort',
            ),
            StartupStep(
                name='qr_vision_node',
                command=['ros2', 'launch', 'drone_perception', 'industrial_animal_vision.launch.py'],
                # 视觉伺服目标话题以 SensorDataQoS(best_effort) 发布；
                # 探针必须用 best_effort 订阅，reliable 会因 QoS 不兼容而永远收不到。
                ready_topic='/vision/servo/target',
                ready_type='drone_msgs/msg/VisionServoTarget',
                timeout_sec=20,
                ready_qos_reliability='best_effort',
            ),
            StartupStep(
                name='airborne_link_bridge',
                command=['ros2', 'run', 'drone_qt_2', 'airborne_link_bridge'],
                ready_topic='/mavros/state',
                ready_type='mavros_msgs/msg/State',
                timeout_sec=15,
                ready_qos_reliability='best_effort',
            ),
            StartupStep(
                name='compare_yaw',
                command=['ros2', 'run', 'drone_localization', 'compare_yaw_node'],
                ready_topic='/pose_yaw_compare/delta',
                ready_type='geometry_msgs/msg/Vector3',
                timeout_sec=15,
                ready_qos_reliability='reliable',
            ),
            # StartupStep(
            #     name='airborne_node',
            #     command=['ros2', 'run', 'drone_qt_2', 'airborne_node'],
            #     ready_topic='/drone/status',
            #     ready_type='drone_msgs/msg/DroneStatus',
            #     timeout_sec=15,
            #     ready_qos_reliability='best_effort',
            # ),
        ]

    def log(self, message: str):
        now = time.strftime('%H:%M:%S')
        print(f'[{now}] {message}', flush=True)

    def handle_signal(self, signum, frame):
        self.log(f'Received signal {signum}, stopping all processes...')
        self.running = False
        self.stop_all_processes()
        sys.exit(0)

    def start_process(self, step: StartupStep) -> subprocess.Popen:
        self.log(f'Starting step: {step.name}')
        process = subprocess.Popen(step.command)
        self.processes.append((step.name, process))
        return process

    def wait_for_ready(self, step: StartupStep) -> bool:
        self.log(f'Waiting ready for {step.name}: topic={step.ready_topic}, type={step.ready_type}, timeout={step.timeout_sec}s')

        result = subprocess.run([
            'python3',
            self.wait_script,
            '--topic', step.ready_topic,
            '--type', step.ready_type,
            '--timeout', str(step.timeout_sec),
            '--qos-reliability', step.ready_qos_reliability,
            '--stable-sec', str(step.ready_stable_sec),
            '--min-messages', str(step.ready_min_messages),
            '--max-gap-sec', str(step.ready_max_gap_sec),
        ])

        return result.returncode == 0

    def stop_all_processes(self):
        for name, process in reversed(self.processes):
            if process.poll() is None:
                self.log(f'Stopping process: {name}')
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.log(f'Force killing process: {name}')
                    process.kill()
                    process.wait(timeout=2)
        self.processes.clear()

    def monitor_processes_forever(self):
        self.log('All steps are ready. Entering monitor mode.')
        while self.running:
            for name, process in self.processes:
                rc = process.poll()
                if rc is not None:
                    self.log(f'Process exited unexpectedly: {name}, returncode={rc}')
                    self.stop_all_processes()
                    sys.exit(1)
            time.sleep(1.0)

    def run(self):
        try:
            for step in self.steps:
                if not self.running:
                    break

                process = self.start_process(step)
                time.sleep(1.0)

                if process.poll() is not None:
                    self.log(f'Process exited too early: {step.name}, returncode={process.returncode}')
                    self.stop_all_processes()
                    sys.exit(1)

                ready = self.wait_for_ready(step)
                if not ready:
                    self.log(f'Ready check failed: {step.name}')
                    self.stop_all_processes()
                    sys.exit(1)

                self.log(f'Step ready: {step.name}')

                if step.post_ready_delay_sec > 0.0:
                    self.log(f'Settling after {step.name}: {step.post_ready_delay_sec}s')
                    time.sleep(step.post_ready_delay_sec)

            self.monitor_processes_forever()
        except Exception as exc:
            self.log(f'Fatal error: {exc}')
            self.stop_all_processes()
            sys.exit(1)


def main():
    supervisor = StartupSupervisor()
    supervisor.run()


if __name__ == '__main__':
    main()
