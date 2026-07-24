#!/usr/bin/env python3
import argparse
import os
import signal
import sys
import threading
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
    parser.add_argument("--visible-height", type=int, default=0,
                        help="Visible image height when decoder output is aligned")
    parser.add_argument("--topic", default="/rdk_video/image")
    parser.add_argument("--window", default="RDK Video")
    parser.add_argument("--no-ros", action="store_true")
    parser.add_argument("--ros-every", type=int, default=2,
                        help="Publish every Nth frame when ROS has subscribers")
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

    visible_height = args.visible_height if args.visible_height > 0 else args.height
    if visible_height > args.height:
        raise ValueError("--visible-height cannot be larger than --height")

    frame_bytes = args.width * args.height * 3 // 2
    frame_count = 0
    start = time.monotonic()
    fps_window_start = start
    fps_window_count = 0
    display_fps = 0.0
    stop = False
    latest = [None]
    latest_lock = threading.Lock()
    reader_done = threading.Event()

    def on_signal(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    def drain_fifo():
        try:
            with open(args.input, "rb", buffering=0) as stream:
                while not stop:
                    payload = read_exact(stream, frame_bytes)
                    if payload is None:
                        break
                    with latest_lock:
                        latest[0] = payload
        finally:
            reader_done.set()

    reader = threading.Thread(target=drain_fifo, name="nv12-reader", daemon=True)
    reader.start()

    try:
        cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
        while not stop:
            with latest_lock:
                payload = latest[0]
                latest[0] = None
            if payload is None:
                if reader_done.is_set():
                    break
                cv2.waitKey(1)
                time.sleep(0.001)
                continue
            nv12 = np.frombuffer(payload, dtype=np.uint8).reshape((args.height * 3 // 2, args.width))
            bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
            if visible_height != args.height:
                bgr = bgr[:visible_height, :]
            frame_count += 1
            fps_window_count += 1
            now = time.monotonic()
            fps_elapsed = now - fps_window_start
            if fps_elapsed >= 0.5:
                display_fps = fps_window_count / fps_elapsed
                fps_window_start = now
                fps_window_count = 0
            cv2.putText(
                bgr,
                f"RDK VPU  {display_fps:.1f} FPS  frames={frame_count}  {args.width}x{visible_height}",
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(args.window, bgr)

            if (publisher is not None
                    and frame_count % max(args.ros_every, 1) == 0
                    and publisher.get_subscription_count() > 0):
                message = ros[2]()
                message.height = visible_height
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
