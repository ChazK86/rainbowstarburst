"""Command-line endpoint and feature diagnostic; never records audio."""

from __future__ import annotations

import argparse
import time

from .devices import AudioDeviceManager, BACKEND_NAME, HAS_WASAPI_LOOPBACK
from .engine import AudioEngine, AudioSourceMode


def list_devices() -> None:
    systems, microphones = AudioDeviceManager().enumerate()
    print(f"Backend: {BACKEND_NAME}; WASAPI loopback: {HAS_WASAPI_LOOPBACK}")
    print("\nSystem output loopbacks:")
    for device in systems:
        marker = " [default]" if device.is_default else ""
        print(f"  {device.identifier}: {device.name} ({device.sample_rate} Hz, {device.channels} ch){marker}")
    if not systems:
        print("  none")
    print("\nMicrophones:")
    for device in microphones:
        marker = " [default]" if device.is_default else ""
        print(f"  {device.identifier}: {device.name} ({device.sample_rate} Hz, {device.channels} ch){marker}")
    if not microphones:
        print("  none")


def monitor(source: str, seconds: float) -> None:
    mode = AudioSourceMode(source.upper())
    engine = AudioEngine()
    try:
        engine.configure(mode)
        print("Live analysis only. No audio is being recorded or transmitted.")
        deadline = time.monotonic() + max(0.1, seconds)
        while time.monotonic() < deadline:
            frame = engine.snapshot()
            status = engine.status()
            print(
                f"\rseq={frame.sequence:6d}  system={frame.system.rms:0.3f}  "
                f"mic={frame.microphone.rms:0.3f}  latency={frame.capture_latency_ms:5.1f} ms  "
                f"drops={frame.dropped_windows}  "
                f"[{status['system'].state}/{status['microphone'].state}]",
                end="",
                flush=True,
            )
            time.sleep(0.1)
        print()
    finally:
        engine.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="list capture endpoints")
    parser.add_argument(
        "--source",
        choices=("system", "mic", "both", "demo"),
        help="monitor normalized levels from a source",
    )
    parser.add_argument("--seconds", type=float, default=10.0, help="monitor duration")
    args = parser.parse_args()
    if args.list or not args.source:
        list_devices()
    if args.source:
        monitor(args.source, args.seconds)


if __name__ == "__main__":
    main()
