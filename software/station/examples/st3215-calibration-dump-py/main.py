"""Dump ST3215 calibration state (range_min/range_max/offset) for debugging.

Read-only — never writes to a motor. Useful for comparing the calibrated
arc (range_min/range_max, used by station-viewer to render the URDF) against
the physical servo's EEPROM position offset (register 0x1F), when the real
arm's rest pose doesn't match what's rendered.

Usage:
  python main.py [--server localhost] [--bus auto] [--watch]
"""

import argparse
import asyncio
import logging
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent.parent))

from software.station.shared.station_py import new_station_client
from target.gen_python.protobuf.drivers.st3215 import st3215

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(message)s")
logger = logging.getLogger(__name__)

# Register addresses inside motor.get_state() bytes.
# Source: software/drivers/st3215/src/protocol/memory.rs
EEPROM_OFFSET = 0x1F           # 2 bytes, signed
RAM_PRESENT_POSITION = 0x38    # 2 bytes

MAX_ANGLE_STEP = 4095
SIGN_BIT_MASK = 0x8000
FULL_RANGE = 4096


def _normal_position(raw: int) -> int:
    """Strip the sign bit so positions are always 0-4095."""
    if raw & SIGN_BIT_MASK:
        magnitude = raw & MAX_ANGLE_STEP
        return (MAX_ANGLE_STEP + 1 - magnitude) & MAX_ANGLE_STEP
    return raw & MAX_ANGLE_STEP


def get_present_position(state: bytes) -> int:
    if len(state) < RAM_PRESENT_POSITION + 2:
        return 0
    raw = struct.unpack_from("<H", state, RAM_PRESENT_POSITION)[0]
    return _normal_position(raw)


def get_offset(state: bytes) -> int:
    """EEPROM position offset (signed). Write-only from the station's point
    of view (only `freeze_calibration` sets it) but readable here since the
    full register map is mirrored into every inference frame."""
    if len(state) < EEPROM_OFFSET + 2:
        return 0
    return struct.unpack_from("<h", state, EEPROM_OFFSET)[0]


def compute_midpoint(range_min: int, range_max: int) -> int:
    """Same formula the calibrator uses to center an arc (handles wraparound)."""
    if range_max >= range_min:
        return range_min + (range_max - range_min) // 2
    range_size = (FULL_RANGE - range_min) + range_max
    return (range_min + range_size // 2) & 0xFFF


def resolve_bus_serial(inference_state, requested: str) -> str:
    buses = inference_state.get_buses() or []
    if not buses:
        raise RuntimeError("No ST3215 buses on station")

    if requested == "auto":
        if len(buses) != 1:
            serials = [b.get_bus().get_serial_number() for b in buses if b.get_bus()]
            raise RuntimeError(
                f"--bus auto requires exactly one bus, found {len(buses)}: {serials}"
            )
        info = buses[0].get_bus()
        if info is None:
            raise RuntimeError("Bus has no info")
        return info.get_serial_number()

    for bus in buses:
        info = bus.get_bus()
        if info and info.get_serial_number() == requested:
            return requested
    raise RuntimeError(f"Bus '{requested}' not found on station")


def find_bus(inference_state, bus_serial: str):
    for bus in inference_state.get_buses() or []:
        info = bus.get_bus()
        if info and info.get_serial_number() == bus_serial:
            return bus
    return None


def print_snapshot(bus_serial: str, bus) -> None:
    motors = sorted(bus.get_motors() or [], key=lambda m: m.get_id())
    print(f"\nBus {bus_serial}  ({len(motors)} motors)")
    print(f"{'id':>3}  {'position':>8}  {'offset':>7}  {'range_min':>9}  {'range_max':>9}  {'midpoint':>8}  {'frozen':>6}")
    for motor in motors:
        state_bytes = bytes(motor.get_state())
        position = get_present_position(state_bytes)
        offset = get_offset(state_bytes)
        range_min = motor.get_range_min()
        range_max = motor.get_range_max()
        midpoint = compute_midpoint(range_min, range_max)
        frozen = motor.get_range_freezed()
        print(
            f"{motor.get_id():>3}  {position:>8}  {offset:>7}  {range_min:>9}  "
            f"{range_max:>9}  {midpoint:>8}  {str(frozen):>6}"
        )


async def main_async(server: str, bus_requested: str, watch: bool):
    logger.info("Connecting to %s...", server)
    client = await new_station_client(server, logger)
    logger.info("Connected")

    inference_queue = asyncio.Queue()
    error_queue = client.follow("st3215/inference", inference_queue)

    bus_serial = None
    try:
        while True:
            if not error_queue.empty():
                raise RuntimeError(f"Inference stream error: {error_queue.get_nowait()}")

            entry = await asyncio.wait_for(inference_queue.get(), timeout=5.0)
            if entry is None:
                raise RuntimeError("Inference stream closed")

            state = st3215.InferenceStateReader(entry.Data)

            if bus_serial is None:
                bus_serial = resolve_bus_serial(state, bus_requested)
                logger.info("Using bus: %s", bus_serial)

            bus = find_bus(state, bus_serial)
            if bus is None:
                continue

            print_snapshot(bus_serial, bus)

            if not watch:
                return
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass


def main():
    parser = argparse.ArgumentParser(
        description="Dump ST3215 calibration state (range/offset) for debugging"
    )
    parser.add_argument("--server", default="localhost", help="Station server address")
    parser.add_argument("--bus", default="auto", help="Bus serial ('auto' = only bus on station)")
    parser.add_argument(
        "--watch", action="store_true", help="Keep printing on every frame instead of exiting after one"
    )
    args = parser.parse_args()
    asyncio.run(main_async(args.server, args.bus, args.watch))


if __name__ == "__main__":
    main()
