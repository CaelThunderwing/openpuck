#!/usr/bin/env python3
"""Update firmware-branch capability metadata from decoded UF2 payloads."""

import argparse
import json
from pathlib import Path
import struct

BLOCK_SIZE = 512
PAYLOAD_OFFSET = 32
TRAILER_OFFSET = 508
MAGIC_START0 = 0x0A324655
MAGIC_START1 = 0x9E5D5157
MAGIC_END = 0x0AB16F30
FWUP_TAG = b"OPK-FWUP-v1"


def uf2_payload_blocks(path):
    data = path.read_bytes()

    if len(data) == 0 or len(data) % BLOCK_SIZE != 0:
        raise SystemExit(
            f"Invalid UF2 length for {path}: {len(data)} bytes"
        )

    blocks = []
    declared_count = None
    seen_numbers = set()

    for offset in range(0, len(data), BLOCK_SIZE):
        block = data[offset:offset + BLOCK_SIZE]
        (
            magic0,
            magic1,
            flags,
            target,
            payload_size,
            block_number,
            block_count,
            family,
        ) = struct.unpack_from("<8I", block)

        if magic0 != MAGIC_START0 or magic1 != MAGIC_START1:
            raise SystemExit(
                f"Invalid UF2 header in {path}, block {offset // BLOCK_SIZE}"
            )

        trailer = struct.unpack_from("<I", block, TRAILER_OFFSET)[0]
        if trailer != MAGIC_END:
            raise SystemExit(
                f"Invalid UF2 trailer in {path}, block {offset // BLOCK_SIZE}"
            )

        if payload_size == 0 or PAYLOAD_OFFSET + payload_size > TRAILER_OFFSET:
            raise SystemExit(
                f"Invalid UF2 payload size in {path}: {payload_size}"
            )

        if declared_count is None:
            declared_count = block_count
        elif block_count != declared_count:
            raise SystemExit(f"Inconsistent UF2 block count in {path}")

        if block_number >= block_count:
            raise SystemExit(
                f"Invalid UF2 block number in {path}: {block_number}"
            )

        if block_number in seen_numbers:
            raise SystemExit(
                f"Duplicate UF2 block number in {path}: {block_number}"
            )

        seen_numbers.add(block_number)
        payload = block[
            PAYLOAD_OFFSET:PAYLOAD_OFFSET + payload_size
        ]
        blocks.append((target, payload))

    if declared_count != len(seen_numbers):
        raise SystemExit(
            f"Incomplete UF2 in {path}: "
            f"declared {declared_count}, found {len(seen_numbers)}"
        )

    return sorted(blocks, key=lambda item: item[0])


def uf2_contains(path, needle):
    tail = b""
    previous_end = None

    for target, payload in uf2_payload_blocks(path):
        if previous_end != target:
            tail = b""

        combined = tail + payload
        if needle in combined:
            return True

        tail = combined[-(len(needle) - 1):]
        previous_end = target + len(payload)

    return False


def update_manifest(directory):
    directory = directory.resolve()
    manifest_path = directory / "manifest.json"

    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        if not isinstance(manifest, dict):
            raise SystemExit(f"Invalid manifest object: {manifest_path}")
    else:
        manifest = {}

    images = sorted(directory.glob("OpenPuck-*.uf2"))
    if not images:
        raise SystemExit(f"No OpenPuck UF2 images found in: {directory}")

    for image in images:
        supported = uf2_contains(image, FWUP_TAG)
        manifest[image.name] = {"panelUpdate": supported}
        state = "true" if supported else "false"
        print(f"{image.name}: panelUpdate={state}")

    manifest_path.write_text(
        json.dumps(manifest, indent=1, sort_keys=True) + "\n"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "directory",
        type=Path,
        help="directory containing mirrored OpenPuck UF2 images",
    )
    args = parser.parse_args()
    update_manifest(args.directory)


if __name__ == "__main__":
    main()
