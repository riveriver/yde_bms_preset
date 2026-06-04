#!/usr/bin/env python3
"""
YDE BMS preset tool over Modbus RTU or Modbus TCP.

Examples:
  python bms_preset.py
  python bms_preset.py factory --mode rtu --port COM3
  python bms_preset.py get-address --mode rtu --port COM3
  python bms_preset.py set-address --mode tcp --ip 192.168.1.20 --uid 2 --address 3
  python bms_preset.py set-capacity --mode tcp --ip 192.168.1.20 --uid 2 --capacity 40
  python bms_preset.py set-capacity --mode rtu --port COM3 --capacity 40
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from dataclasses import dataclass


BROADCAST_READ_ADDR = 0xFF
TARGET_SLAVE_ADDR = 0x02
ADDR_REGISTER = 0x0000
CAPACITY_REGISTER = 0x0068
CAPACITY_40AH_VALUE = 0x0190

FACTORY_SLAVE_ADDR = 0x02
FACTORY_CAPACITY_AH = 40.0

COMMANDS = ("factory", "get-address", "set-address", "set-capacity", "preset")
COMMAND_LABELS = {
    "factory": "Set factory parameters",
    "get-address": "Read BMS address by broadcast",
    "set-address": "Set BMS address",
    "set-capacity": "Set nominal capacity",
    "preset": "Set address, then set nominal capacity",
}


class BmsError(RuntimeError):
    pass


@dataclass
class ConnectionConfig:
    mode: str
    serial_port: str | None
    tcp_ip: str | None
    tcp_port: int
    uid: int | None
    baudrate: int
    parity: str
    stopbits: float
    bytesize: int
    timeout: float
    retries: int
    settle_delay: float
    dry_run: bool


def modbus_crc(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def add_crc(frame_without_crc: bytes) -> bytes:
    crc = modbus_crc(frame_without_crc)
    return frame_without_crc + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def hex_frame(frame: bytes) -> str:
    return "-".join(f"{byte:02X}" for byte in frame)


def print_line() -> None:
    print("-" * 56)


def print_frame(direction: str, frame: bytes) -> None:
    label = "TX" if direction.upper() == "TX" else "RX"
    print(f"  {label:<3}: {hex_frame(frame)}")


def print_connection_config(config: ConnectionConfig) -> None:
    run_mode = "DRY RUN" if config.dry_run else "LIVE"
    print_line()
    print(f"Mode   : Modbus {config.mode.upper()}  {run_mode}")
    if config.mode == "rtu":
        print(f"Serial : {config.serial_port}  {config.baudrate} {config.bytesize}{config.parity}{config.stopbits:g}")
    else:
        print(f"TCP    : {config.tcp_ip}:{config.tcp_port}")
        print(f"UID    : {config.uid} (0x{config.uid:02X})")
    print_line()


def build_read_input_registers_pdu(register: int, count: int) -> bytes:
    return bytes(
        (0x04, (register >> 8) & 0xFF, register & 0xFF, (count >> 8) & 0xFF, count & 0xFF)
    )


def build_write_single_register_pdu(register: int, value: int) -> bytes:
    return bytes((0x06, (register >> 8) & 0xFF, register & 0xFF, (value >> 8) & 0xFF, value & 0xFF))


def next_transaction_id() -> int:
    next_id = getattr(next_transaction_id, "_value", 0) + 1
    if next_id > 0xFFFF:
        next_id = 1
    setattr(next_transaction_id, "_value", next_id)
    return next_id


def build_request(config: ConnectionConfig, slave_addr: int, pdu: bytes) -> bytes:
    if config.mode == "rtu":
        return add_crc(bytes((slave_addr,)) + pdu)

    transaction_id = next_transaction_id()
    length = 1 + len(pdu)
    mbap = bytes(
        (
            (transaction_id >> 8) & 0xFF,
            transaction_id & 0xFF,
            0x00,
            0x00,
            (length >> 8) & 0xFF,
            length & 0xFF,
            slave_addr,
        )
    )
    return mbap + pdu


def validate_response_crc(response: bytes) -> None:
    if len(response) < 4:
        raise BmsError(f"response too short: {hex_frame(response)}")

    expected = response[-2] | (response[-1] << 8)
    actual = modbus_crc(response[:-2])
    if actual != expected:
        raise BmsError(
            f"CRC mismatch: RX={hex_frame(response)}, calculated CRC={actual:04X}"
        )


def open_serial(config: ConnectionConfig):
    try:
        import serial
    except ImportError as exc:
        raise BmsError(
            "pyserial is not installed. Install it with: python -m pip install pyserial"
        ) from exc

    return serial.Serial(
        port=config.serial_port,
        baudrate=config.baudrate,
        parity=config.parity,
        stopbits=config.stopbits,
        bytesize=config.bytesize,
        timeout=config.timeout,
    )


def strip_and_validate_response(config: ConnectionConfig, response: bytes) -> bytes:
    if config.mode == "rtu":
        validate_response_crc(response)
        return response[:-2]

    if len(response) < 9:
        raise BmsError(f"TCP response too short: {hex_frame(response)}")

    protocol_id = (response[2] << 8) | response[3]
    if protocol_id != 0:
        raise BmsError(f"unexpected TCP protocol id: {protocol_id}")

    length = (response[4] << 8) | response[5]
    expected_len = 6 + length
    if len(response) != expected_len:
        raise BmsError(
            f"unexpected TCP response length: got {len(response)}, expected {expected_len}"
        )
    return response[6:]


def recv_modbus_tcp(sock: socket.socket) -> bytes:
    header = sock.recv(7)
    if not header:
        return b""
    while len(header) < 7:
        chunk = sock.recv(7 - len(header))
        if not chunk:
            break
        header += chunk

    if len(header) < 7:
        return header

    length = (header[4] << 8) | header[5]
    remaining = max(0, length - 1)
    body = b""
    while len(body) < remaining:
        chunk = sock.recv(remaining - len(body))
        if not chunk:
            break
        body += chunk
    return header + body


def transact(config: ConnectionConfig, request: bytes, expected_len: int | None = None) -> bytes:
    print_frame("TX", request)

    if config.dry_run:
        print("  RX : skipped in dry-run mode")
        return b""

    last_response = b""
    for attempt in range(1, config.retries + 1):
        if config.retries > 1:
            print(f"  Attempt: {attempt}/{config.retries}")

        if config.mode == "rtu":
            response = transact_rtu(config, request, expected_len)
        else:
            response = transact_tcp(config, request)

        if response:
            print_frame("RX", response)
            return strip_and_validate_response(config, response)

        last_response = response
        if attempt < config.retries:
            time.sleep(config.settle_delay)

    raise BmsError(f"no response after {config.retries} attempt(s): {hex_frame(last_response)}")


def transact_rtu(
    config: ConnectionConfig,
    request: bytes,
    expected_len: int | None = None,
) -> bytes:
    with open_serial(config) as ser:
            ser.reset_input_buffer()
            ser.write(request)
            ser.flush()
            time.sleep(config.settle_delay)
            return ser.read(256 if expected_len is None else expected_len)


def transact_tcp(config: ConnectionConfig, request: bytes) -> bytes:
    if config.tcp_ip is None:
        raise BmsError("TCP IP address is not configured")

    with socket.create_connection((config.tcp_ip, config.tcp_port), timeout=config.timeout) as sock:
        sock.settimeout(config.timeout)
        sock.sendall(request)
        return recv_modbus_tcp(sock)


def parse_address_response(response: bytes) -> int:
    if len(response) != 5:
        raise BmsError(f"unexpected address response length: {hex_frame(response)}")
    if response[1] & 0x80:
        raise BmsError(f"Modbus exception response: {hex_frame(response)}")
    if response[1] != 0x04 or response[2] != 0x02:
        raise BmsError(f"unexpected address response: {hex_frame(response)}")
    return (response[3] << 8) | response[4]


def ensure_write_echo(config: ConnectionConfig, request: bytes, response: bytes) -> None:
    expected = request[:-2] if config.mode == "rtu" else request[6:]
    if response != expected:
        raise BmsError(
            f"write echo mismatch: expected={hex_frame(expected)}, RX={hex_frame(response)}"
        )


def get_address(config: ConnectionConfig) -> int | None:
    print("")
    print("Step: read address register by broadcast")

    pdu = build_read_input_registers_pdu(ADDR_REGISTER, 1)
    request = build_request(config, BROADCAST_READ_ADDR, pdu)
    response = transact(config, request, expected_len=7)
    if config.dry_run:
        print("Result: dry-run only; address was not read.")
        return None

    address = parse_address_response(response)
    print(f"Result: current BMS address is {address} (0x{address:02X}).")
    return address


def set_address(config: ConnectionConfig, target_address: int = TARGET_SLAVE_ADDR) -> None:
    print("")
    print(f"Step: ensure BMS address is {target_address} (0x{target_address:02X})")

    if config.mode == "tcp":
        if config.uid is None:
            raise BmsError("TCP UID is not configured")
        current_uid = config.uid
        print("")
        print(f"Step: write address register from {current_uid} to {target_address}")
        pdu = build_write_single_register_pdu(ADDR_REGISTER, target_address)
        request = build_request(config, current_uid, pdu)
        response = transact(config, request, expected_len=8)
        if config.dry_run:
            print("Result: dry-run only; address was not written.")
            return
        ensure_write_echo(config, request, response)
        config.uid = target_address
        print(f"Result: address changed from {current_uid} to {target_address}.")
        return

    current_address = get_address(config)
    if current_address is None:
        pdu = build_write_single_register_pdu(ADDR_REGISTER, target_address)
        request = build_request(config, target_address, pdu)
        print("")
        print("Dry-run note: current address is unknown, so this write frame is only a template.")
        print_frame("TX", request)
        print("  RX : skipped in dry-run mode")
        return

    if current_address == target_address:
        print(f"Result: address is already {target_address}; no write needed.")
        return

    print("")
    print(f"Step: write address register from {current_address} to {target_address}")

    pdu = build_write_single_register_pdu(ADDR_REGISTER, target_address)
    request = build_request(config, current_address, pdu)
    response = transact(config, request, expected_len=8)
    ensure_write_echo(config, request, response)
    print(f"Result: address changed from {current_address} to {target_address}.")


def prompt_command(config: ConnectionConfig) -> str:
    if config.mode == "tcp":
        menu_items = (
            ("1", "set-address", "Set slave address by user input"),
            ("2", "set-capacity", "Set nominal capacity by user input"),
            ("0", "exit", "Exit"),
        )
    else:
        menu_items = (
            ("1", "factory", "Set factory parameters: address 2, capacity 40Ah"),
            ("2", "get-address", "Read slave address by broadcast"),
            ("3", "set-address", "Set slave address by user input"),
            ("4", "set-capacity", "Set nominal capacity by user input"),
            ("0", "exit", "Exit"),
        )

    print("")
    print_line()
    print("BMS preset menu")
    print_line()
    for number, _, label in menu_items:
        print(f"  {number}. {label}")

    command_by_choice = {number: command for number, command, _ in menu_items}
    command_by_choice.update({command: command for _, command, _ in menu_items})
    command_by_choice["q"] = "exit"
    command_by_choice["quit"] = "exit"

    while True:
        choice = input("Select option: ").strip()
        command = command_by_choice.get(choice)
        if command:
            return command
        print("Invalid option. Please try again.")


def prompt_port() -> str:
    print("")
    print_line()
    print("Serial port setup")
    print_line()
    print("No serial port was provided.")
    print("Examples: COM3 on Windows, /dev/ttyUSB0 on Linux.")

    while True:
        port = input("Serial port: ").strip()
        if port:
            return port
        print("Serial port cannot be empty.")


def prompt_mode() -> str:
    print("")
    print_line()
    print("Communication mode")
    print_line()
    print("  1. Modbus RTU")
    print("  2. Modbus TCP")

    while True:
        mode = input("Select mode [1/rtu, 2/tcp]: ").strip().lower()
        if mode in ("1", "rtu"):
            return "rtu"
        if mode in ("2", "tcp"):
            return "tcp"
        print("Invalid mode. Please try again.")


def prompt_ip() -> str:
    print("")
    print_line()
    print("TCP setup")
    print_line()
    print("No IP address was provided.")

    while True:
        ip = input("IP address: ").strip()
        if ip:
            return ip
        print("IP address cannot be empty.")


def prompt_tcp_port(default: int = 502) -> int:
    raw = input(f"TCP port [{default}]: ").strip()
    if not raw:
        return default
    try:
        port = int(raw, 0)
    except ValueError as exc:
        raise BmsError(f"invalid TCP port: {raw}") from exc
    if not 1 <= port <= 65535:
        raise BmsError(f"TCP port must be between 1 and 65535: {port}")
    return port


def prompt_uid() -> int:
    return prompt_int("TCP UID (1-247): ", 1, 247)


def build_config(args: argparse.Namespace) -> ConnectionConfig:
    mode = args.mode
    if mode is None and args.ip:
        mode = "tcp"
    if mode is None and args.port:
        mode = "rtu"
    if mode is None:
        mode = prompt_mode()

    serial_port = None
    tcp_ip = None
    tcp_port = args.tcp_port or 502
    uid = None

    if mode == "rtu":
        serial_port = args.port or prompt_port()
    else:
        tcp_ip = args.ip or prompt_ip()
        if not args.command and args.tcp_port is None:
            tcp_port = prompt_tcp_port(502)
        elif not 1 <= tcp_port <= 65535:
            raise BmsError(f"TCP port must be between 1 and 65535: {tcp_port}")
        uid = args.uid if args.uid is not None else prompt_uid()
        if not 1 <= uid <= 247:
            raise BmsError(f"TCP UID must be between 1 and 247: {uid}")

    config = ConnectionConfig(
        mode=mode,
        serial_port=serial_port,
        tcp_ip=tcp_ip,
        tcp_port=tcp_port,
        uid=uid,
        baudrate=args.baudrate,
        parity=args.parity,
        stopbits=args.stopbits,
        bytesize=args.bytesize,
        timeout=args.timeout,
        retries=args.retries,
        settle_delay=args.settle_delay,
        dry_run=args.dry_run,
    )
    print_connection_config(config)
    return config


def prompt_int(prompt: str, min_value: int, max_value: int) -> int:
    while True:
        raw = input(prompt).strip()
        try:
            value = int(raw, 0)
        except ValueError:
            print("Invalid number. Please try again.")
            continue

        if min_value <= value <= max_value:
            return value
        print(f"Value must be between {min_value} and {max_value}.")


def prompt_float(prompt: str, min_value: float, max_value: float) -> float:
    while True:
        raw = input(prompt).strip()
        try:
            value = float(raw)
        except ValueError:
            print("Invalid number. Please try again.")
            continue

        if min_value <= value <= max_value:
            return value
        print(f"Value must be between {min_value:g} and {max_value:g}.")


def capacity_ah_to_register_value(capacity_ah: float) -> int:
    value = int(round(capacity_ah * 10))
    if not 0 <= value <= 0xFFFF:
        raise BmsError(f"capacity is out of register range: {capacity_ah:g}Ah")
    return value


def set_capacity_value(
    config: ConnectionConfig,
    capacity_ah: float,
    slave_addr: int | None = None,
) -> None:
    register_value = capacity_ah_to_register_value(capacity_ah)
    if slave_addr is None:
        slave_addr = config.uid if config.mode == "tcp" else TARGET_SLAVE_ADDR
    if slave_addr is None:
        raise BmsError("slave address is not configured")

    print("")
    print("Step: write nominal capacity")

    pdu = build_write_single_register_pdu(CAPACITY_REGISTER, register_value)
    request = build_request(config, slave_addr, pdu)
    response = transact(config, request, expected_len=8)
    if config.dry_run:
        print("Result: dry-run only; capacity was not written.")
        return

    ensure_write_echo(config, request, response)
    print(f"Result: nominal capacity set to {capacity_ah:g}Ah.")


def run_factory(config: ConnectionConfig) -> None:
    set_address(config, FACTORY_SLAVE_ADDR)
    if not config.dry_run:
        time.sleep(config.settle_delay)
    set_capacity_value(config, FACTORY_CAPACITY_AH, FACTORY_SLAVE_ADDR)


def run_command(
    command: str,
    config: ConnectionConfig,
    address: int | None = None,
    capacity_ah: float | None = None,
) -> None:
    print("")
    print_line()
    print(f"Operation: {COMMAND_LABELS.get(command, command)}")
    print_line()

    if command == "factory":
        if config.mode == "tcp":
            raise BmsError("factory setup is not available in Modbus TCP mode")
        run_factory(config)
    elif command == "get-address":
        if config.mode == "tcp":
            raise BmsError("broadcast address discovery is not available in Modbus TCP mode")
        get_address(config)
    elif command == "set-address":
        if address is None:
            address = prompt_int("Target slave address (1-247): ", 1, 247)
        elif not 1 <= address <= 247:
            raise BmsError(f"target slave address must be between 1 and 247: {address}")
        set_address(config, address)
    elif command == "set-capacity":
        if capacity_ah is None:
            capacity_ah = prompt_float("Nominal capacity in Ah: ", 0.1, 6553.5)
        set_capacity_value(config, capacity_ah, TARGET_SLAVE_ADDR)
    elif command == "preset":
        if config.mode == "tcp":
            raise BmsError("preset is not available in Modbus TCP mode")
        run_factory(config)
    else:
        raise BmsError(f"unknown command: {command}")


def interactive_loop(config: ConnectionConfig) -> None:
    while True:
        command = prompt_command(config)
        if command == "exit":
            print("Exit.")
            return

        try:
            run_command(command, config)
        except BmsError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Configure YDE BMS slave address and nominal capacity over Modbus RTU or TCP."
    )
    parser.add_argument(
        "command",
        nargs="?",
        choices=COMMANDS,
        help=(
            "factory writes default parameters; get-address reads address by broadcast; "
            "set-address writes a user-provided address; set-capacity writes a "
            "user-provided nominal capacity."
        ),
    )
    parser.add_argument("--mode", choices=("rtu", "tcp"), help="Communication mode.")
    parser.add_argument("--port", help="RTU serial port, for example COM3.")
    parser.add_argument("--ip", help="Modbus TCP IP address.")
    parser.add_argument(
        "--uid",
        type=lambda value: int(value, 0),
        help="Modbus TCP Unit ID, required in TCP mode, for example 2 or 0x02.",
    )
    parser.add_argument(
        "--tcp-port",
        type=int,
        default=None,
        help="Modbus TCP port. Default: 502.",
    )
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        help="Target slave address for set-address, for example 2 or 0x02.",
    )
    parser.add_argument(
        "--capacity",
        type=float,
        help="Nominal capacity in Ah for set-capacity, for example 40.",
    )
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--parity", choices=("N", "E", "O"), default="N")
    parser.add_argument("--stopbits", type=float, choices=(1.0, 1.5, 2.0), default=1.0)
    parser.add_argument("--bytesize", type=int, choices=(7, 8), default=8)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--retries", type=int, default=3)
    parser.add_argument("--settle-delay", type=float, default=0.1)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Only print generated TX frames; do not open the serial port.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = build_config(args)
    except BmsError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    if args.command:
        try:
            run_command(
                args.command,
                config,
                address=args.address,
                capacity_ah=args.capacity,
            )
        except BmsError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 1
    else:
        interactive_loop(config)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
