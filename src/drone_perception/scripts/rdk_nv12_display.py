#!/usr/bin/env python3
import argparse
import os
import signal
import sys
import time

import cv2
import numpy as np


def read_exact(stream, size):
    chunks = []
    remaining = size
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def main():
    parser = argparse.ArgumentParser(description="Display RDK VPU NV12 FIFO and publish sensor_msgs/Image")
    parser.add_argument("--input", required=True, help="NV12 FIFO or file")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--topic", default="/rdk_video/image")
    parser.add_argument("--window", default="RDK Video")
    parser.add_argument("--no-ros", action="store_true")
    args = parser.parse_args()

    if not os.path.exists(args.input):
        raise FileNotFoundError(args.input)

    ros = None
    publisher = None
    if not args.no_ros:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image

        rclpy.init()
        node = Node("rdk_video_display")
        publisher = node.create_publisher(Image, args.topic, 1)
        ros = (rclpy, node, Image)
    else:
        node = None

    frame_bytes = args.width * args.height * 3 // 2
    frame_count = 0
    start = time.monotonic()
    stop = False

    def on_signal(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        with open(args.input, "rb", buffering=0) as stream:
            cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
            while not stop:
                payload = read_exact(stream, frame_bytes)
                if payload is None:
                    break
                nv12 = np.frombuffer(payload, dtype=np.uint8).reshape((args.height * 3 // 2, args.width))
                bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
                frame_count += 1
                elapsed = max(time.monotonic() - start, 1e-6)
                fps = frame_count / elapsed
                cv2.putText(
                    bgr,
                    f"RDK VPU  {fps:.1f} FPS  frames={frame_count}  {args.width}x{args.height}",
                    (16, 32),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow(args.window, bgr)

                if publisher is not None:
                    message = ros[2]()
                    message.height = args.height
                    message.width = args.width
                    message.encoding = "bgr8"
                    message.is_bigendian = False
                    message.step = args.width * 3
                    message.data = bgr.tobytes()
                    publisher.publish(message)
                    ros[0].spin_once(node, timeout_sec=0.0)

                if cv2.waitKey(1) & 0xFF == 27:
                    break
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
            ros[0].shutdown()

    print(f"displayed_frames={frame_count}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
