#!/usr/bin/env python3
"""Receive the CanMV stage-1 UDP heartbeat and report loss/freshness."""

import argparse
import json
import socket
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5601)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()

    received = 0
    malformed = 0
    lost = 0
    last_seq = None
    started = time.monotonic()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((args.host, args.port))
        sock.settimeout(args.timeout)
        print(f"[READY] K230 link probe listening on {args.host}:{args.port}", flush=True)

        while True:
            try:
                data, address = sock.recvfrom(4096)
            except socket.timeout:
                print(
                    f"[TIMEOUT] no heartbeat for {args.timeout:.1f}s "
                    f"received={received} lost={lost} malformed={malformed}",
                    flush=True,
                )
                continue

            now = time.monotonic()
            try:
                payload = json.loads(data.decode("utf-8"))
                if payload.get("type") != "k230_link_probe":
                    raise ValueError(f"unexpected type={payload.get('type')}")
                sequence = int(payload["seq"])
            except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
                malformed += 1
                print(f"[BAD] from={address} bytes={len(data)} error={exc}", flush=True)
                continue

            if last_seq is not None and sequence > last_seq + 1:
                lost += sequence - last_seq - 1
            if last_seq is not None and sequence <= last_seq:
                print(f"[WARN] out-of-order seq={sequence} previous={last_seq}", flush=True)
            last_seq = max(sequence, last_seq if last_seq is not None else sequence)
            received += 1
            elapsed = max(now - started, 1e-6)
            print(
                "[RX] "
                f"from={address[0]} seq={sequence} rate={received / elapsed:.2f}pps "
                f"lost={lost} camera_fps={payload.get('camera_fps')} "
                f"k230_ip={payload.get('ip')} heap={payload.get('free_heap')}",
                flush=True,
            )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
