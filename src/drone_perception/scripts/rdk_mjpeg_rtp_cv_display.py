#!/usr/bin/env python3
import argparse
import signal
import sys
import time

import cv2
import numpy as np

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description="Display RTP/JPEG UDP stream with OpenCV")
    parser.add_argument("--port", type=int, default=5004)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--window", default="RDK MJPEG RTP")
    args = parser.parse_args()

    Gst.init(None)
    caps = (
        "application/x-rtp,"
        "media=(string)video,"
        "clock-rate=(int)90000,"
        "encoding-name=(string)JPEG,"
        "payload=(int)26,"
        f"width=(int){args.width},"
        f"height=(int){args.height}"
    )
    pipeline_desc = (
        f"udpsrc port={args.port} buffer-size=262144 caps=\"{caps}\" ! "
        "rtpjpegdepay ! jpegdec ! videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true"
    )
    pipeline = Gst.parse_launch(pipeline_desc)
    sink = pipeline.get_by_name("sink")
    stop = False

    def on_signal(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    pipeline.set_state(Gst.State.PLAYING)
    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(args.window, args.width, args.height)

    frame_count = 0
    fps_window_count = 0
    fps_window_start = time.monotonic()
    display_fps = 0.0

    try:
        while not stop:
            sample = sink.emit("try-pull-sample", 100_000_000)
            if sample is None:
                cv2.waitKey(1)
                continue

            buffer = sample.get_buffer()
            ok, map_info = buffer.map(Gst.MapFlags.READ)
            if not ok:
                continue
            try:
                frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape(
                    (args.height, args.width, 3)
                )
                bgr = frame.copy()
            finally:
                buffer.unmap(map_info)

            frame_count += 1
            fps_window_count += 1
            now = time.monotonic()
            elapsed = now - fps_window_start
            if elapsed >= 0.5:
                display_fps = fps_window_count / elapsed
                fps_window_start = now
                fps_window_count = 0

            cv2.putText(
                bgr,
                f"MJPEG RTP  {display_fps:.1f} FPS  frames={frame_count}  {args.width}x{args.height}",
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(args.window, bgr)
            if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                break
    finally:
        pipeline.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()

    print(f"displayed_frames={frame_count}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
