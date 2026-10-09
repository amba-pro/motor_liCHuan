#!/usr/bin/env python3
"""Prepare one Profile Position jog. Refuses until confirmation and every latch pass.

This command does not run unless --arm and --confirm MOVE are both present,
and even then the safety latches and position scaling still have to pass.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import config
import nic
from safety import hardware_blockers
from servo_controller import LichuanServo, SafetyBlocked, _print_status


def confirmation_blockers(arm: bool, confirm: str) -> list[str]:
    blockers = []
    if not arm:
        blockers.append("pass --arm")
    if confirm != "MOVE":
        blockers.append("pass --confirm MOVE")
    return blockers


def preflight(degrees: float, speed_rpm: float, arm: bool, confirm: str) -> list[str]:
    """Blockers that can be decided before the bus is opened."""
    blockers = confirmation_blockers(arm, confirm)
    blockers.extend(hardware_blockers())
    if abs(degrees) > config.MAX_MOVE_DEGREES:
        blockers.append(f"requested {degrees} deg exceeds MAX_MOVE_DEGREES={config.MAX_MOVE_DEGREES}")
    if speed_rpm <= 0 or speed_rpm > config.MAX_SPEED_RPM:
        blockers.append(f"speed {speed_rpm} rpm is outside 0 < speed <= {config.MAX_SPEED_RPM}")
    return blockers


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Latched LC-E Profile Position jog.")
    parser.add_argument("--degrees", type=float, required=True)
    parser.add_argument("--speed-rpm", type=float, default=config.MAX_SPEED_RPM)
    parser.add_argument("--arm", action="store_true")
    parser.add_argument("--confirm", default="", help="must be the exact word MOVE")
    args = parser.parse_args(argv)
    blockers = preflight(args.degrees, args.speed_rpm, args.arm, args.confirm)
    if blockers:
        print("JOG REFUSED")
        for item in blockers:
            print(f"  - {item}")
        return 2 if confirmation_blockers(args.arm, args.confirm) else 3
    nic.ensure_root()
    servo = LichuanServo()
    try:
        servo.connect()
        try:
            before = servo.read_position()
            servo.move_relative(args.degrees, args.speed_rpm)
            if before is not None:
                _print_status(servo._move_to(before, args.speed_rpm))
            servo.disable()
            return 0
        except SafetyBlocked as exc:
            print("JOG REFUSED")
            for item in exc.blockers:
                print(f"  - {item}")
            return 3
    finally:
        servo.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
