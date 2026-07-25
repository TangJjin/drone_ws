#!/usr/bin/env python3

import sys
import time
import argparse
import importlib

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def import_message_type(type_string: str):
    parts = type_string.split('/')
    if len(parts) != 3 or parts[1] != 'msg':
        raise ValueError(f'Invalid message type: {type_string}')

    package_name = parts[0]
    message_name = parts[2]

    module = importlib.import_module(f'{package_name}.msg')
    return getattr(module, message_name)


class TopicWaiter(Node):
    def __init__(
    self,
    topic_name: str,
    msg_type,
    timeout_sec: float,
    qos_reliability: str,
    stable_sec: float,
    min_messages: int,
    max_gap_sec: float,
    ):
        super().__init__('topic_waiter')
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.timeout_sec = timeout_sec
        self.stable_sec = stable_sec
        self.min_messages = min_messages
        self.max_gap_sec = max_gap_sec
        self.received = False
        self.message_count = 0
        self.stable_start_time = None
        self.last_message_time = None
        self.start_time = time.monotonic()

        reliability_map = {
            'reliable': QoSReliabilityPolicy.RELIABLE,
            'best_effort': QoSReliabilityPolicy.BEST_EFFORT,
        }
        if qos_reliability not in reliability_map:
            raise ValueError(f'Invalid qos reliability: {qos_reliability}')

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=reliability_map[qos_reliability],
        )

        self.subscription = self.create_subscription(
            self.msg_type,
            self.topic_name,
            self.callback,
            qos
        )

    def callback(self, msg):
        now = time.monotonic()

        if self.last_message_time is None or (now - self.last_message_time) > self.max_gap_sec:
            self.stable_start_time = now
            self.message_count = 0

        self.received = True
        self.message_count += 1
        self.last_message_time = now

        if self.message_count == 1:
            self.get_logger().info(f'Received first message from {self.topic_name}')

    def is_timeout(self) -> bool:
        return (time.monotonic() - self.start_time) >= self.timeout_sec

    def is_stable(self) -> bool:
        if not self.received or self.stable_start_time is None or self.last_message_time is None:
            return False

        now = time.monotonic()

        if (now - self.last_message_time) > self.max_gap_sec:
            self.received = False
            self.message_count = 0
            self.stable_start_time = None
            self.get_logger().warning(
                f'Topic stream gap exceeded {self.max_gap_sec:.2f}s: {self.topic_name}'
            )
            return False

        return (
            self.message_count >= self.min_messages and
            (now - self.stable_start_time) >= self.stable_sec
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stable-sec', type=float, default=1.0)
    parser.add_argument('--min-messages', type=int, default=3)
    parser.add_argument('--max-gap-sec', type=float, default=1.5)

    parser.add_argument('--topic', required=True, help='topic name')
    parser.add_argument('--type', required=True, help='message type, e.g. std_msgs/msg/String')
    parser.add_argument('--timeout', type=float, default=15.0, help='timeout in seconds')
    parser.add_argument(
        '--qos-reliability',
        choices=['reliable', 'best_effort'],
        default='reliable',
        help='subscription QoS reliability'
    )
    args = parser.parse_args()

    try:
        msg_type = import_message_type(args.type)
    except Exception as exc:
        print(f'Failed to import message type {args.type}: {exc}', file=sys.stderr)
        sys.exit(2)

    rclpy.init()
    node = TopicWaiter(
        args.topic,
        msg_type,
        args.timeout,
        args.qos_reliability,
        args.stable_sec,
        args.min_messages,
        args.max_gap_sec,
    )

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)

            if node.is_stable():
                node.get_logger().info(f'Topic ready: {args.topic}')
                node.destroy_node()
                rclpy.shutdown()
                sys.exit(0)

            if node.is_timeout():
                node.get_logger().error(f'Timeout waiting for topic: {args.topic}')
                node.destroy_node()
                rclpy.shutdown()
                sys.exit(1)
    except KeyboardInterrupt:
        node.get_logger().warning('Interrupted while waiting for topic')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(130)


if __name__ == '__main__':
    main()
