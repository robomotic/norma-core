"""Minimal connectivity check for a remote station.

Connects to a station's NormFS TCP server, follows an inference queue, and
prints a summary of the first frame received, then keeps printing all motor
positions once a second until interrupted. Useful for confirming that a
station on another host is reachable and actively publishing before writing
anything more involved against it.

Usage:
    uv run python connect.py --host 192.168.68.66
    uv run python connect.py --host 192.168.68.66 --queue st3215/inference --timeout 10
"""

import argparse
import asyncio
import logging
import struct
import sys
from pathlib import Path

# Add repo root so the generated protobuf and station_py imports resolve.
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent.parent))

from software.station.shared.station_py import new_station_client
from target.gen_python.protobuf.drivers.st3215 import st3215

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(message)s")
logger = logging.getLogger(__name__)

# Position field inside motor.get_state() bytes: 2 bytes, high bit is a sign flag.
PRESENT_POSITION_ADDR = 0x38
MAX_ANGLE_STEP = 4095
SIGN_BIT_MASK = 0x8000

PRINT_INTERVAL_S = 1.0


def _normal_position(raw: int) -> int:
    """Strip the sign bit so positions are always 0-4095."""
    if raw & SIGN_BIT_MASK:
        magnitude = raw & MAX_ANGLE_STEP
        return (MAX_ANGLE_STEP + 1 - magnitude) & MAX_ANGLE_STEP
    return raw & MAX_ANGLE_STEP


def _motor_position(state_bytes: bytes) -> int:
    if len(state_bytes) < PRESENT_POSITION_ADDR + 2:
        return 0
    raw = struct.unpack_from("<H", state_bytes, PRESENT_POSITION_ADDR)[0]
    return _normal_position(raw)


def _print_positions(state) -> None:
    buses = state.get_buses() or []
    for bus in buses:
        info = bus.get_bus()
        serial = info.get_serial_number() if info else "?"
        motors = bus.get_motors() or []
        positions = ", ".join(
            f"{m.get_id()}={_motor_position(bytes(m.get_state()))}" for m in motors
        )
        logger.info("  bus=%s positions: %s", serial, positions or "(no motors)")


async def check_connection(host: str, queue_id: str, timeout: float):
    client = await new_station_client(host, logger)
    logger.info("Connected to %s, setup done: %s", host, client.setup_done)

    entries_queue = asyncio.Queue()
    error_queue = client.follow(queue_id, entries_queue)
    logger.info("Following %s, waiting up to %.0fs for a frame...", queue_id, timeout)

    try:
        entry = await asyncio.wait_for(entries_queue.get(), timeout=timeout)
    except asyncio.TimeoutError:
        if not error_queue.empty():
            raise RuntimeError(f"follow error: {await error_queue.get()}")
        raise RuntimeError(
            f"no frame received within {timeout:.0f}s "
            "(queue may be idle, or the producing driver isn't enabled)"
        )

    if entry is None:
        raise RuntimeError("stream closed immediately")

    data = bytes(entry.Data)
    logger.info("Got frame: %d bytes", len(data))

    state = st3215.InferenceStateReader(memoryview(data))
    buses = state.get_buses() or []
    logger.info("Buses reported: %d", len(buses))
    for bus in buses:
        info = bus.get_bus()
        motors = bus.get_motors() or []
        serial = info.get_serial_number() if info else "?"
        port = info.get_port_name() if info else "?"
        logger.info("  bus serial=%s port=%s motors=%d", serial, port, len(motors))

    logger.info("Printing motor positions every %.0fs (Ctrl+C to stop)...", PRINT_INTERVAL_S)
    latest = state
    while True:
        try:
            while not entries_queue.empty():
                next_entry = entries_queue.get_nowait()
                if next_entry is None:
                    raise RuntimeError("stream closed")
                latest = st3215.InferenceStateReader(memoryview(bytes(next_entry.Data)))
        except asyncio.QueueEmpty:
            pass

        _print_positions(latest)
        await asyncio.sleep(PRINT_INTERVAL_S)


def main():
    parser = argparse.ArgumentParser(description="Check connectivity to a station")
    parser.add_argument("--host", default="localhost", help="Station hostname or IP (default: localhost)")
    parser.add_argument("--queue", default="st3215/inference", help="Queue to follow (default: st3215/inference)")
    parser.add_argument("--timeout", type=float, default=10.0, help="Seconds to wait for a frame (default: 10)")
    args = parser.parse_args()

    try:
        asyncio.run(check_connection(args.host, args.queue, args.timeout))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
