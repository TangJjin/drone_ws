#!/usr/bin/env python3

import os
import sys
import time
import signal
import subprocess
from dataclasses import dataclass
from typing import List, Optional


USB_RESET_COMMAND = '/usr/bin/usbreset'


@dataclass
class StartupStep:
    name: str
    command: List[str]
    ready_topic: str
    ready_type: str
    timeout_sec: int
    ready_qos_reliability: str = 'reliable'
    ready_stable_sec: float = 2.0
    ready_min_messages: int = 5
    ready_max_gap_sec: float = 2.0
    post_ready_delay_sec: float = 1.0
    max_attempts: int = 1
    recovery_usb_id: Optional[str] = None
    recovery_settle_sec: float = 0.0


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
                command=[
                    'ros2', 'launch', 'drone_perception',
                    'industrial_animal_vision.launch.py',
                ],
                # 视觉伺服目标话题以 SensorDataQoS(best_effort) 发布；
                # 探针必须用 best_effort 订阅，reliable 会因 QoS 不兼容而永远收不到。
                ready_topic='/vision/servo/target',
                ready_type='drone_msgs/msg/VisionServoTarget',
                timeout_sec=20,
                ready_qos_reliability='best_effort',
                max_attempts=2,
                recovery_usb_id='32e4:6577',
                recovery_settle_sec=3.0,
            ),
            # StartupStep(
            #     name='airborne_link_bridge',
            #     command=['ros2', 'run', 'drone_qt_2', 'airborne_link_bridge'],
            #     ready_topic='/mavros/state',
            #     ready_type='mavros_msgs/msg/State',
            #     timeout_sec=15,
            #     ready_qos_reliability='best_effort',
            # ),
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
        process = subprocess.Popen(
            step.command,
            start_new_session=(os.name == 'posix'),
        )
        self.processes.append((step.name, process))
        return process

    def wait_for_ready(
        self,
        step: StartupStep,
        process: subprocess.Popen,
    ) -> bool:
        self.log(
            f'Waiting ready for {step.name}: topic={step.ready_topic}, '
            f'type={step.ready_type}, timeout={step.timeout_sec}s'
        )

        waiter = subprocess.Popen([
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

        try:
            while self.running:
                waiter_rc = waiter.poll()
                if waiter_rc is not None:
                    return waiter_rc == 0

                process_rc = process.poll()
                if process_rc is not None:
                    self.log(
                        f'Process exited while waiting ready: {step.name}, '
                        f'returncode={process_rc}'
                    )
                    return False

                time.sleep(0.1)
            return False
        finally:
            if waiter.poll() is None:
                waiter.terminate()
                try:
                    waiter.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    waiter.kill()
                    waiter.wait(timeout=2)

    def signal_process(self, process: subprocess.Popen, signum: int):
        try:
            if os.name == 'posix':
                os.killpg(process.pid, signum)
            elif signum == signal.SIGKILL:
                process.kill()
            else:
                process.terminate()
        except ProcessLookupError:
            pass

    def stop_process(
        self,
        name: str,
        process: subprocess.Popen,
        remove: bool = True,
    ):
        if process.poll() is None:
            self.log(f'Stopping process group: {name}')
            self.signal_process(process, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.log(f'Terminating process group: {name}')
                self.signal_process(process, signal.SIGTERM)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.log(f'Force killing process group: {name}')
                    self.signal_process(process, signal.SIGKILL)
                    process.wait(timeout=2)

        if remove:
            self.processes = [
                item for item in self.processes if item[1] is not process
            ]

    def reset_usb_device(self, usb_id: str) -> bool:
        self.log(f'Resetting USB device: {usb_id}')
        try:
            result = subprocess.run(
                [USB_RESET_COMMAND, usb_id],
                capture_output=True,
                text=True,
                timeout=10,
                check=False,
            )
        except FileNotFoundError:
            self.log(f'USB reset command not found: {USB_RESET_COMMAND}')
            return False
        except subprocess.TimeoutExpired:
            self.log(f'USB reset timed out: {usb_id}')
            return False

        stdout = result.stdout.strip().replace('\n', ' | ')
        stderr = result.stderr.strip().replace('\n', ' | ')
        self.log(
            f'USB reset finished: id={usb_id}, returncode={result.returncode}, '
            f'stdout={stdout or "<empty>"}, stderr={stderr or "<empty>"}'
        )
        return result.returncode == 0

    def start_step_attempt(
        self,
        step: StartupStep,
        attempt: int,
    ) -> Optional[subprocess.Popen]:
        if step.max_attempts > 1:
            self.log(
                f'Starting attempt {attempt}/{step.max_attempts}: {step.name}'
            )

        process = self.start_process(step)
        time.sleep(1.0)

        if process.poll() is not None:
            self.log(
                f'Process exited too early: {step.name}, '
                f'returncode={process.returncode}'
            )
            self.stop_process(step.name, process)
            return None

        if not self.wait_for_ready(step, process):
            self.log(f'Ready check failed: {step.name}')
            self.stop_process(step.name, process)
            return None

        if process.poll() is not None:
            self.log(
                f'Process exited after ready check: {step.name}, '
                f'returncode={process.returncode}'
            )
            self.stop_process(step.name, process)
            return None

        return process

    def recover_step(self, step: StartupStep) -> bool:
        if step.recovery_usb_id is None:
            return False
        if not self.reset_usb_device(step.recovery_usb_id):
            return False

        if step.recovery_settle_sec > 0.0:
            self.log(
                f'Waiting for USB device to settle: '
                f'{step.recovery_settle_sec}s'
            )
            time.sleep(step.recovery_settle_sec)
        return self.running

    def stop_all_processes(self):
        for name, process in reversed(self.processes.copy()):
            self.stop_process(name, process, remove=False)
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

                process = None
                for attempt in range(1, step.max_attempts + 1):
                    process = self.start_step_attempt(step, attempt)
                    if process is not None:
                        break

                    if attempt >= step.max_attempts:
                        break

                    self.log(
                        f'Recovering failed step before retry: {step.name}'
                    )
                    if not self.recover_step(step):
                        self.log(f'Recovery failed: {step.name}')
                        break

                if process is None:
                    self.log(f'Startup failed after retries: {step.name}')
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
