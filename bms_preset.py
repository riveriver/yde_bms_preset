#!/usr/bin/env python3
"""
YDE BMS preset tool over Modbus RTU.

Examples:
  python bms_preset.py
  python bms_preset.py factory --port COM3
  python bms_preset.py get-address --port COM3
  python bms_preset.py set-address --port COM3 --address 2
  python bms_preset.py set-capacity --port COM3 --capacity 40
  python bms_preset.py preset --port COM3
"""

from __future__ import annotations

import argparse
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


@dataclass(frozen=True)
class SerialConfig:
    port: str
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


def print_serial_config(config: SerialConfig) -> None:
    mode = "DRY RUN" if config.dry_run else "LIVE"
    print_line()
    print(f"Serial : {config.port}  {config.baudrate} {config.bytesize}{config.parity}{config.stopbits:g}")
    print(f"Mode   : {mode}")
    print_line()


def build_read_input_registers(slave_addr: int, register: int, count: int) -> bytes:
    body = bytes(
        (
            slave_addr,
            0x04,
            (register >> 8) & 0xFF,
            register & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
        )
    )
    return add_crc(body)


def build_write_single_register(slave_addr: int, register: int, value: int) -> bytes:
    body = bytes(
        (
            slave_addr,
            0x06,
            (register >> 8) & 0xFF,
            register & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        )
    )
    return add_crc(body)


def validate_response_crc(response: bytes) -> None:
    if len(response) < 4:
        raise BmsError(f"response too short: {hex_frame(response)}")

    expected = response[-2] | (response[-1] << 8)
    actual = modbus_crc(response[:-2])
    if actual != expected:
        raise BmsError(
            f"CRC mismatch: RX={hex_frame(response)}, calculated CRC={actual:04X}"
        )


def open_serial(config: SerialConfig):
    try:
        import serial
    except ImportError as exc:
        raise BmsError(
            "pyserial is not installed. Install it with: python -m pip install pyserial"
        ) from exc

    return serial.Serial(
        port=config.port,
        baudrate=config.baudrate,
        parity=config.parity,
        stopbits=config.stopbits,
        bytesize=config.bytesize,
        timeout=config.timeout,
    )


def transact(config: SerialConfig, request: bytes, expected_len: int | None = None) -> bytes:
    print_frame("TX", request)

    if config.dry_run:
        print("  RX : skipped in dry-run mode")
        return b""

    last_response = b""
    with open_serial(config) as ser:
        for attempt in range(1, config.retries + 1):
            if config.retries > 1:
                print(f"  Attempt: {attempt}/{config.retries}")

            ser.reset_input_buffer()
            ser.write(request)
            ser.flush()
            time.sleep(config.settle_delay)

            response = ser.read(256 if expected_len is None else expected_len)
            if response:
                print_frame("RX", response)
                validate_response_crc(response)
                return response

            last_response = response
            if attempt < config.retries:
                time.sleep(config.settle_delay)

    raise BmsError(f"no response after {config.retries} attempt(s): {hex_frame(last_response)}")


def parse_address_response(response: bytes) -> int:
    if len(response) != 7:
        raise BmsError(f"unexpected address response length: {hex_frame(response)}")
    if response[1] & 0x80:
        raise BmsError(f"Modbus exception response: {hex_frame(response)}")
    if response[1] != 0x04 or response[2] != 0x02:
        raise BmsError(f"unexpected address response: {hex_frame(response)}")
    return (response[3] << 8) | response[4]


def ensure_write_echo(request: bytes, response: bytes) -> None:
    if response != request:
        raise BmsError(
            f"write echo mismatch: TX={hex_frame(request)}, RX={hex_frame(response)}"
        )


def get_address(config: SerialConfig) -> int | None:
    print("")
    print("Step: read address register by broadcast")

    request = build_read_input_registers(BROADCAST_READ_ADDR, ADDR_REGISTER, 1)
    response = transact(config, request, expected_len=7)
    if config.dry_run:
        print("Result: dry-run only; address was not read.")
        return None

    address = parse_address_response(response)
    print(f"Result: current BMS address is {address} (0x{address:02X}).")
    return address


def set_address(config: SerialConfig, target_address: int = TARGET_SLAVE_ADDR) -> None:
    print("")
    print(f"Step: ensure BMS address is {target_address} (0x{target_address:02X})")

    current_address = get_address(config)
    if current_address is None:
        request = build_write_single_register(target_address, ADDR_REGISTER, target_address)
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

    request = build_write_single_register(current_address, ADDR_REGISTER, target_address)
    response = transact(config, request, expected_len=8)
    ensure_write_echo(request, response)
    print(f"Result: address changed from {current_address} to {target_address}.")


def prompt_command() -> str:
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


def build_config(args: argparse.Namespace) -> SerialConfig:
    port = args.port or prompt_port()
    config = SerialConfig(
        port=port,
        baudrate=args.baudrate,
        parity=args.parity,
        stopbits=args.stopbits,
        bytesize=args.bytesize,
        timeout=args.timeout,
        retries=args.retries,
        settle_delay=args.settle_delay,
        dry_run=args.dry_run,
    )
    print_serial_config(config)
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
    config: SerialConfig,
    capacity_ah: float,
    slave_addr: int = TARGET_SLAVE_ADDR,
) -> None:
    register_value = capacity_ah_to_register_value(capacity_ah)

    print("")
    print("Step: write nominal capacity")

    request = build_write_single_register(slave_addr, CAPACITY_REGISTER, register_value)
    response = transact(config, request, expected_len=8)
    if config.dry_run:
        print("Result: dry-run only; capacity was not written.")
        return

    ensure_write_echo(request, response)
    print(f"Result: nominal capacity set to {capacity_ah:g}Ah.")


def run_factory(config: SerialConfig) -> None:
    set_address(config, FACTORY_SLAVE_ADDR)
    if not config.dry_run:
        time.sleep(config.settle_delay)
    set_capacity_value(config, FACTORY_CAPACITY_AH, FACTORY_SLAVE_ADDR)


def run_command(
    command: str,
    config: SerialConfig,
    address: int | None = None,
    capacity_ah: float | None = None,
) -> None:
    print("")
    print_line()
    print(f"Operation: {COMMAND_LABELS.get(command, command)}")
    print_line()

    if command == "factory":
        run_factory(config)
    elif command == "get-address":
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
        run_factory(config)
    else:
        raise BmsError(f"unknown command: {command}")


def interactive_loop(config: SerialConfig) -> None:
    while True:
        command = prompt_command()
        if command == "exit":
            print("Exit.")
            return

        try:
            run_command(command, config)
        except BmsError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Configure YDE BMS slave address and nominal capacity over Modbus RTU."
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
    parser.add_argument("--port", help="Serial port, for example COM3.")
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
    config = build_config(args)

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
