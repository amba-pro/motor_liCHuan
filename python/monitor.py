#!/usr/bin/env python3
"""Print drive feedback until Ctrl-C. Does not enable the servo."""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import nic
from servo_controller import LichuanServo


def main() -> int:
    nic.ensure_root()
    servo = LichuanServo()
    try:
        servo.connect()
        print("monitoring; Ctrl-C to stop. The servo is not enabled.")
        while True:
            status = servo.read_status()
            print(
                f"state={status['cia402']} ecat={status['ethercat_state']} "
                f"error={status['error']} mode={status['mode']} "
                f"pos={status['position']} vel={status['velocity']} torque={status['torque']}",
                flush=True,
            )
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nmonitor stopped")
        return 0
    finally:
        servo.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
