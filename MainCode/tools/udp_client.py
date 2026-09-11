#!/usr/bin/env python3
"""Minimal dependency-free client for the Qiming UWV UDP protocol."""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
import zlib
from dataclasses import dataclass


MAGIC = b"UW"
VERSION = 1
PORT = 20001
HEADER = struct.Struct("<2sBBHHI")
CRC = struct.Struct("<I")
MOTOR_PAYLOAD = struct.Struct("<BBBBfffffHH")
REPLY_PAYLOAD = struct.Struct("<BBH")
TELEMETRY_PAYLOAD = struct.Struct("<III6fIBBHI4B4f")
ARM_STATUS_PAYLOAD = struct.Struct("<IBBH3f5fffIfff")

MSG_HELLO = 0x01
MSG_MOTOR = 0x10
MSG_SAFE_STOP = 0x11
MSG_FORCE = 0x20
MSG_TELEMETRY = 0x80
MSG_ACK = 0x81
MSG_ERROR = 0x82
MSG_ARM_STATUS = 0x83

ARM_OPTION = 0x8000

STATUS_NAMES = {
    0: "OK",
    1: "BAD_FRAME",
    2: "BAD_COMMAND",
    3: "SAFETY_LOCKED",
    4: "UNSUPPORTED",
    5: "IO_ERROR",
    6: "REPLAY",
    7: "NOT_READY",
}

DRIVERS = {"erob": 1, "robstride": 2}
OPERATIONS = {
    "disable": 0,
    "enable": 1,
    "stop": 2,
    "zero": 3,
    "position": 4,
    "velocity": 5,
    "torque": 6,
    "impedance": 7,
    "fault-reset": 8,
}
SAFE_OPERATIONS = {"disable", "stop", "fault-reset"}
FORCE_COMMANDS = {
    "stop": 0x01,
    "stream": 0x02,
    "once": 0x03,
    "baud": 0x04,
    "serial": 0x05,
    "initialize": 0x06,
    "version": 0x07,
    "tare": 0x30,
    "debug-off": 0x31,
    "debug-on": 0x32,
    "kg": 0x33,
    "raw": 0x34,
    "newton": 0x35,
}
BAUD_CODES = {460800: 1, 691200: 2, 921600: 3}


@dataclass(frozen=True)
class Packet:
    message_type: int
    flags: int
    sequence: int
    payload: bytes


def encode_packet(message_type: int, sequence: int, payload: bytes = b"") -> bytes:
    if len(payload) > 256:
        raise ValueError("payload exceeds 256 bytes")
    body = HEADER.pack(MAGIC, VERSION, message_type, 0, len(payload), sequence)
    body += payload
    return body + CRC.pack(zlib.crc32(body) & 0xFFFFFFFF)


def decode_packet(datagram: bytes) -> Packet:
    if len(datagram) < HEADER.size + CRC.size:
        raise ValueError("short datagram")
    magic, version, message_type, flags, length, sequence = HEADER.unpack_from(
        datagram
    )
    if magic != MAGIC or version != VERSION:
        raise ValueError("bad magic or protocol version")
    expected = HEADER.size + length + CRC.size
    if len(datagram) != expected:
        raise ValueError(f"bad length: received {len(datagram)}, expected {expected}")
    received_crc = CRC.unpack_from(datagram, len(datagram) - CRC.size)[0]
    computed_crc = zlib.crc32(datagram[:-CRC.size]) & 0xFFFFFFFF
    if received_crc != computed_crc:
        raise ValueError("bad CRC32")
    return Packet(message_type, flags, sequence, datagram[HEADER.size:-CRC.size])


def print_telemetry(packet: Packet) -> None:
    if len(packet.payload) != TELEMETRY_PAYLOAD.size:
        print(f"telemetry with unexpected length {len(packet.payload)}")
        return
    values = TELEMETRY_PAYLOAD.unpack(packet.payload)
    (
        board_ms,
        flags,
        force_ms,
        fx,
        fy,
        fz,
        mx,
        my,
        mz,
        erob_ms,
        erob_node,
        erob_state,
        erob_status,
        rob_ms,
        rob_id,
        rob_state,
        rob_faults,
        rob_model,
        rob_pos,
        rob_vel,
        rob_torque,
        rob_temp,
    ) = values
    lock = "locked" if flags & (1 << 8) else "armed-build"
    print(
        f"t={board_ms:10d} ms flags=0x{flags:08X} {lock} | "
        f"force@{force_ms}: [{fx:+.4f}, {fy:+.4f}, {fz:+.4f}; "
        f"{mx:+.4f}, {my:+.4f}, {mz:+.4f}] | "
        f"eRob@{erob_ms}: id={erob_node} state={erob_state} "
        f"sw=0x{erob_status:04X} | RobStride@{rob_ms}: id={rob_id} "
        f"model={rob_model} state={rob_state} fault=0x{rob_faults:02X} "
        f"p={rob_pos:+.4f} v={rob_vel:+.4f} tq={rob_torque:+.4f} "
        f"temp={rob_temp:.1f} C"
    )


def print_arm_status(packet: Packet) -> None:
    if len(packet.payload) != ARM_STATUS_PAYLOAD.size:
        print(f"arm status with unexpected length {len(packet.payload)}")
        return
    values = ARM_STATUS_PAYLOAD.unpack(packet.payload)
    board_ms, state, ik_status, current_ma = values[:4]
    target = values[4:7]
    joints = values[7:12]
    gripper, force, last_command, pos_error, ori_error, force_limit = values[12:]
    print(
        f"arm t={board_ms:10d} state={state} ik={ik_status} "
        f"target=({target[0]:+.1f},{target[1]:+.1f},{target[2]:+.1f}) mm "
        f"q={[round(q, 3) for q in joints]} grip={gripper:+.3f} rad "
        f"force={force:.2f}/{force_limit:.2f} N Imax={current_ma} mA "
        f"err=({pos_error:.2f} mm,{ori_error:.4f} rad) last={last_command}"
    )


def receive_until_reply(
    sock: socket.socket, expected_sequence: int, timeout: float
) -> Packet:
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("timed out waiting for ACK/ERROR")
        sock.settimeout(remaining)
        datagram, source = sock.recvfrom(2048)
        try:
            packet = decode_packet(datagram)
        except ValueError as error:
            print(f"ignored invalid datagram from {source}: {error}", file=sys.stderr)
            continue
        if packet.message_type == MSG_TELEMETRY:
            print_telemetry(packet)
        elif packet.message_type == MSG_ARM_STATUS:
            print_arm_status(packet)
            continue
        if packet.message_type not in (MSG_ACK, MSG_ERROR):
            continue
        if packet.sequence != expected_sequence:
            continue
        if len(packet.payload) != REPLY_PAYLOAD.size:
            raise ValueError("malformed ACK/ERROR payload")
        request_type, status, detail = REPLY_PAYLOAD.unpack(packet.payload)
        name = STATUS_NAMES.get(status, f"UNKNOWN_{status}")
        print(
            f"{'ACK' if packet.message_type == MSG_ACK else 'ERROR'} "
            f"request=0x{request_type:02X} status={name} detail={detail}"
        )
        if packet.message_type == MSG_ERROR or status != 0:
            raise RuntimeError(name)
        return packet


def send_arm_ascii(
    sock: socket.socket, destination: tuple[str, int], command: bytes, timeout: float
) -> None:
    sock.sendto(command, destination)
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("timed out waiting for ASCII arm reply")
        sock.settimeout(remaining)
        datagram, _ = sock.recvfrom(2048)
        if datagram.startswith(b"ARM:"):
            print(datagram.decode("ascii", errors="replace"))
            return
        try:
            packet = decode_packet(datagram)
        except ValueError:
            continue
        if packet.message_type == MSG_TELEMETRY:
            print_telemetry(packet)
        elif packet.message_type == MSG_ARM_STATUS:
            print_arm_status(packet)


def send_request(
    sock: socket.socket,
    destination: tuple[str, int],
    message_type: int,
    sequence: int,
    payload: bytes,
    timeout: float,
) -> None:
    sock.sendto(encode_packet(message_type, sequence, payload), destination)
    receive_until_reply(sock, sequence, timeout)


def establish_session(
    sock: socket.socket, destination: tuple[str, int], sequence: int, timeout: float
) -> int:
    send_request(sock, destination, MSG_HELLO, sequence, b"", timeout)
    return (sequence + 1) & 0xFFFFFFFF


def listen(sock: socket.socket, seconds: float) -> None:
    deadline = None if seconds <= 0 else time.monotonic() + seconds
    while deadline is None or time.monotonic() < deadline:
        if deadline is not None:
            sock.settimeout(max(0.001, deadline - time.monotonic()))
        else:
            sock.settimeout(None)
        try:
            datagram, source = sock.recvfrom(2048)
        except socket.timeout:
            return
        try:
            packet = decode_packet(datagram)
        except ValueError as error:
            print(f"ignored invalid datagram from {source}: {error}", file=sys.stderr)
            continue
        if packet.message_type == MSG_TELEMETRY:
            print_telemetry(packet)
        elif packet.message_type == MSG_ARM_STATUS:
            print_arm_status(packet)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.10", help="board IPv4 address")
    parser.add_argument("--port", type=int, default=PORT, help="board UDP port")
    parser.add_argument("--bind", default="0.0.0.0", help="local bind address")
    parser.add_argument("--local-port", type=int, default=0, help="local UDP port")
    parser.add_argument("--timeout", type=float, default=1.0, help="reply timeout")
    commands = parser.add_subparsers(dest="command", required=True)

    commands.add_parser("hello", help="start/reset a protocol session")
    commands.add_parser("stop", help="stop all motors tracked by the board")

    listen_parser = commands.add_parser(
        "listen", help="print 50 Hz telemetry and arm status"
    )
    listen_parser.add_argument(
        "--seconds", type=float, default=0.0, help="0 listens until Ctrl+C"
    )

    force_parser = commands.add_parser("force", help="send a force sensor command")
    force_parser.add_argument("action", choices=FORCE_COMMANDS)
    force_parser.add_argument(
        "--baud", type=int, choices=BAUD_CODES, default=460800
    )

    motor = commands.add_parser("motor", help="send one motor command")
    motor.add_argument("--driver", choices=DRIVERS, required=True)
    motor.add_argument("--id", type=int, required=True, dest="node_id")
    motor.add_argument("--op", choices=OPERATIONS, required=True)
    motor.add_argument("--position", type=float, default=0.0)
    motor.add_argument("--velocity", type=float, default=0.0)
    motor.add_argument("--torque", type=float, default=0.0)
    motor.add_argument("--kp", type=float, default=0.0)
    motor.add_argument("--kd", type=float, default=0.0)
    motor.add_argument("--watchdog-ms", type=int, default=200)
    motor.add_argument("--model", type=int, choices=range(7))
    motor.add_argument(
        "--arm", action="store_true", help="required for every hazardous operation"
    )
    arm = commands.add_parser("arm", help="send the NUC ASCII arm command")
    arm.add_argument("action", choices=("position", "gripper", "stop"))
    arm.add_argument("--x", type=int, help="x in 0.01 mm")
    arm.add_argument("--y", type=int, help="y in 0.01 mm")
    arm.add_argument("--z", type=int, help="z in 0.01 mm")
    arm.add_argument("--position", type=int, help="gripper position in 0.001 rad")
    arm.add_argument("--current", type=int, default=1000,
                     help="gripper current in mA")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    destination = (args.host, args.port)
    sequence = (time.time_ns() // 1_000_000) & 0xFFFFFFFF

    if args.command == "motor":
        if not 1 <= args.node_id <= 127:
            parser.error("--id must be in 1..127")
        if not 20 <= args.watchdog_ms <= 5000:
            parser.error("--watchdog-ms must be in 20..5000")
        if args.op not in SAFE_OPERATIONS and not args.arm:
            parser.error("hazardous motor operation requires --arm")
        if args.driver == "erob" and args.model not in (None, 0):
            parser.error("eRob uses --model 0 (model bits are reserved)")

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((args.bind, args.local_port))
        if args.command == "hello":
            send_request(sock, destination, MSG_HELLO, sequence, b"", args.timeout)
            return 0

        sequence = establish_session(sock, destination, sequence, args.timeout)
        if args.command == "arm":
            if args.action == "position":
                if args.x is None or args.y is None or args.z is None:
                    parser.error("arm position requires --x, --y and --z")
                command = (
                    f"PXYZ:{args.x:+07d},{args.y:+07d},{args.z:+07d},/".encode()
                )
            elif args.action == "gripper":
                if args.position is None:
                    parser.error("arm gripper requires --position")
                command = f"JZH_{args.position:+d}_{args.current}/".encode()
            else:
                command = b"ST:P/"
            send_arm_ascii(sock, destination, command, args.timeout)
            return 0
        if args.command == "listen":
            listen(sock, args.seconds)
            return 0
        if args.command == "stop":
            send_request(
                sock, destination, MSG_SAFE_STOP, sequence, b"", args.timeout
            )
            return 0
        if args.command == "force":
            command = FORCE_COMMANDS[args.action]
            payload = bytes([command])
            if args.action == "baud":
                payload += bytes([BAUD_CODES[args.baud]])
            send_request(sock, destination, MSG_FORCE, sequence, payload, args.timeout)
            return 0
        if args.command == "motor":
            driver = DRIVERS[args.driver]
            bus = 1 if args.driver == "erob" else 2
            model = args.model
            if model is None:
                model = 0 if args.driver == "erob" else 1
            options = model
            if args.arm:
                options |= ARM_OPTION
            payload = MOTOR_PAYLOAD.pack(
                bus,
                driver,
                args.node_id,
                OPERATIONS[args.op],
                args.position,
                args.velocity,
                args.torque,
                args.kp,
                args.kd,
                args.watchdog_ms,
                options,
            )
            send_request(sock, destination, MSG_MOTOR, sequence, payload, args.timeout)
            return 0
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, TimeoutError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1)
