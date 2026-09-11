#!/usr/bin/env python3
"""Measure packet integrity, loss, and round-trip time against the board echo test."""

from __future__ import annotations

import argparse
import math
import socket
import statistics
import struct
import time


HEADER = struct.Struct("!4sIQ")
MAGIC = b"QUDP"


def make_payload(sequence: int, size: int) -> bytes:
    sent_ns = time.perf_counter_ns()
    header = HEADER.pack(MAGIC, sequence, sent_ns)
    body = bytes(((sequence + offset) & 0xFF) for offset in range(size - len(header)))
    return header + body


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.10")
    parser.add_argument("--port", type=int, default=20001)
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--size", type=int, default=64)
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--interval", type=float, default=0.01)
    args = parser.parse_args()
    if args.count <= 0 or not HEADER.size <= args.size <= 512:
        parser.error(f"count must be positive and size must be {HEADER.size}..512")
    if args.timeout <= 0.0 or args.interval < 0.0:
        parser.error("timeout must be positive and interval cannot be negative")
    if not 1 <= args.port <= 65535:
        parser.error("port must be 1..65535")
    return args


def main() -> int:
    args = parse_args()
    destination = (socket.gethostbyname(args.host), args.port)
    round_trip_ms: list[float] = []
    lost = 0
    corrupt = 0

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for sequence in range(args.count):
            payload = make_payload(sequence, args.size)
            start_ns = time.perf_counter_ns()
            deadline = time.monotonic() + args.timeout
            sock.sendto(payload, destination)

            matched = False
            while time.monotonic() < deadline:
                sock.settimeout(max(0.001, deadline - time.monotonic()))
                try:
                    response, peer = sock.recvfrom(2048)
                except TimeoutError:
                    break
                if peer != destination or response != payload:
                    corrupt += 1
                    continue
                round_trip_ms.append((time.perf_counter_ns() - start_ns) / 1_000_000.0)
                matched = True
                break
            if not matched:
                lost += 1
            if args.interval > 0.0:
                time.sleep(args.interval)

    received = len(round_trip_ms)
    print(f"sent={args.count} received={received} lost={lost} corrupt={corrupt}")
    if round_trip_ms:
        ordered = sorted(round_trip_ms)
        p95 = ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)]
        print(
            "rtt_ms "
            f"min={ordered[0]:.3f} avg={statistics.mean(ordered):.3f} "
            f"p95={p95:.3f} max={ordered[-1]:.3f}"
        )
    return 0 if lost == 0 and corrupt == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
