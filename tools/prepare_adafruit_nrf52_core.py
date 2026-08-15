#!/usr/bin/env python3
"""Prepare the pinned Adafruit nRF52 core for OpenPuck's EP0 override."""

import argparse
import os
from pathlib import Path
import subprocess
import sys

CORE_VERSION = "1.7.0"
RELATIVE_HEADER = Path(
    "packages/adafruit/hardware/nrf52"
) / CORE_VERSION / (
    "libraries/Adafruit_TinyUSB_Arduino/src/arduino/ports/nrf/"
    "tusb_config_nrf.h"
)

STOCK = """#if CFG_TUD_ENABLED
#define CFG_TUD_ENDPOINT0_SIZE 64
"""

PREPARED = """#if CFG_TUD_ENABLED
#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE 64
#endif
"""


def arduino_data_dir():
    try:
        result = subprocess.run(
            ["arduino-cli", "config", "get", "directories.data"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (FileNotFoundError, subprocess.CalledProcessError) as error:
        raise SystemExit(
            f"Unable to query Arduino data directory: {error}"
        )

    configured = result.stdout.strip()
    if configured:
        return Path(configured).expanduser()

    home = Path.home()
    if os.name == "nt":
        local_app_data = os.environ.get("LOCALAPPDATA")
        if local_app_data:
            return Path(local_app_data) / "Arduino15"
    if sys.platform == "darwin":
        return home / "Library/Arduino15"
    return home / ".arduino15"


def prepare(header):
    if not header.is_file():
        raise SystemExit(
            f"Adafruit nRF52 {CORE_VERSION} header not found: {header}"
        )

    data = header.read_text(encoding="utf-8")

    prepared_count = data.count(PREPARED)
    stock_count = data.count(STOCK)

    if prepared_count == 1 and stock_count == 0:
        print(f"READY: OpenPuck EP0 override guard already present: {header}")
        return

    if prepared_count != 0 or stock_count != 1:
        raise SystemExit(
            "Refusing to modify an unexpected TinyUSB configuration file: "
            f"{header}\n"
            f"stock matches={stock_count}, prepared matches={prepared_count}"
        )

    updated = data.replace(STOCK, PREPARED)
    header.write_text(updated, encoding="utf-8")

    verified = header.read_text(encoding="utf-8")
    if verified.count(PREPARED) != 1 or verified.count(STOCK) != 0:
        raise SystemExit(f"EP0 guard verification failed: {header}")

    print(f"PATCHED: enabled OpenPuck EP0 override in: {header}")


def main():
    parser = argparse.ArgumentParser()
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument(
        "--header",
        type=Path,
        help="prepare this header instead of the installed pinned core",
    )
    selection.add_argument(
        "--print-data-dir",
        action="store_true",
        help="print the resolved Arduino data directory and exit",
    )
    args = parser.parse_args()

    if args.print_data_dir:
        print(arduino_data_dir())
        return

    header = (
        args.header.expanduser()
        if args.header is not None
        else arduino_data_dir() / RELATIVE_HEADER
    )

    prepare(header)


if __name__ == "__main__":
    main()
