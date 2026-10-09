#!/usr/bin/env python3
"""LC-E servo access. Status is read-only. Enable and motion stay latched off."""

from __future__ import annotations

import argparse
import atexit
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pysoem

import config
import nic
from bus import request_preop
from cia402 import (
    OPERATION_ENABLED,
    READY,
    SWITCH_ON_DISABLED,
    SWITCHED_ON,
    counts_to_degrees,
    decode_statusword,
    degrees_to_counts,
    ethercat_state_name,
    confirm_r23_scale,
    evaluate_scaling,
    pack_i32,
    pack_i8,
    pack_u16,
    pack_u32,
)
from pdo import assigned_pdos
from profile_position import (
    PP_MODE,
    absolute_setpoint_words,
    fault_shutdown_words,
    note_wkc,
    profile_units,
    target_reached,
)
from safety import move_blockers, safety_blockers
from sdo import SdoClient
from timing import wait_until


class SafetyBlocked(RuntimeError):
    def __init__(self, blockers: list[str]) -> None:
        self.blockers = blockers
        super().__init__("\n".join(blockers))


class LichuanServo:
    def __init__(self, iface: str = config.IFACE) -> None:
        self.iface = iface
        self.master = None
        self.slave = None
        self.sdo: SdoClient | None = None
        self._managed = False
        self._claimed = False
        self._enabled = False
        self._mapped = False
        self._controlword_bits = None
        self._target_bits = None
        self._target = 0
        self._controlword = 0
        self._installed_hooks = False

    def connect(self) -> int:
        if self.master is not None:
            return len(self.master.slaves)
        self._managed = nic.claim(self.iface)
        self._claimed = True
        self.master = pysoem.Master()
        try:
            self.master.open(self.iface)
            count = self.master.config_init()
        except Exception:
            self.disconnect()
            raise
        if count <= 0:
            self.disconnect()
            raise RuntimeError("no EtherCAT slave")
        request_preop(self.master)
        self.slave = self.master.slaves[0]
        self.sdo = SdoClient(self.slave)
        self._install_hooks()
        return count

    def disconnect(self) -> None:
        if self._enabled:
            try:
                self.disable()
            except Exception:
                self._enabled = False
        if self.master is not None:
            try:
                self.master.state = pysoem.INIT_STATE
                self.master.write_state()
            except Exception:
                pass
            try:
                self.master.close()
            except Exception:
                pass
        self.master = None
        self.slave = None
        self.sdo = None
        if self._claimed:
            nic.release(self.iface, self._managed)
            self._claimed = False

    def _install_hooks(self) -> None:
        if self._installed_hooks:
            return
        atexit.register(self.disconnect)
        signal.signal(signal.SIGINT, self._request_stop)
        signal.signal(signal.SIGTERM, self._request_stop)
        self._installed_hooks = True
        self._stop = False

    def _request_stop(self, _signum, _frame) -> None:
        self._stop = True
        raise KeyboardInterrupt

    def read_error(self) -> int | None:
        value, _err = self.sdo.optional(self.sdo.read_u16, 0x603F, 0)
        return value

    def read_position(self) -> int | None:
        value, _err = self.sdo.optional(self.sdo.read_i32, 0x6064, 0)
        return value

    def read_velocity(self) -> int | None:
        value, _err = self.sdo.optional(self.sdo.read_i32, 0x606C, 0)
        return value

    def read_scaling(self):
        motor, _ = self.sdo.optional(self.sdo.read_u32, 0x6091, 1)
        axis, _ = self.sdo.optional(self.sdo.read_u32, 0x6091, 2)
        inc, _ = self.sdo.optional(self.sdo.read_u32, 0x608F, 1)
        rev, _ = self.sdo.optional(self.sdo.read_u32, 0x608F, 2)
        direct = evaluate_scaling(motor, axis, inc, rev)
        if direct.known:
            return direct
        low, _ = self.sdo.optional(self.sdo.read_u16, 0x2000, 32)
        high, _ = self.sdo.optional(self.sdo.read_u16, 0x2000, 33)
        internal, _ = self.sdo.optional(self.sdo.read_i32, 0x6063, 0)
        actual, _ = self.sdo.optional(self.sdo.read_i32, 0x6064, 0)
        return confirm_r23_scale(motor, axis, low, high, internal, actual)

    def read_status(self) -> dict:
        self.master.read_state()
        status, status_err = self.sdo.optional(self.sdo.read_u16, 0x6041, 0)
        torque, _torque_err = self.sdo.optional(self.sdo.read_i16, 0x6077, 0)
        mode, _mode_err = self.sdo.optional(self.sdo.read_i8, 0x6061, 0)
        return {
            "ethercat_state": ethercat_state_name(self.slave.state),
            "ethercat_raw": self.slave.state,
            "error": self.read_error(),
            "statusword": status,
            "status_error": status_err,
            "cia402": decode_statusword(status) if status is not None else "unreadable",
            "mode": mode,
            "position": self.read_position(),
            "velocity": self.read_velocity(),
            "torque": torque,
            "scaling": self.read_scaling(),
        }

    def _require_ready_for_power(self) -> dict:
        status = self.read_status()
        blockers = safety_blockers(status["scaling"])
        if status["error"]:
            blockers.append(f"active error 0x{status['error']:04X}; no fault reset will be sent")
        if status["position"] is None:
            blockers.append("position feedback 0x6064 is not readable")
        if status["cia402"] in ("Fault", "Fault Reaction Active"):
            blockers.append(f"CiA402 state is {status['cia402']}")
        if (self.slave.state & 0x0F) == 0:
            blockers.append("EtherCAT slave state is not available")
        if blockers:
            raise SafetyBlocked(blockers)
        return status

    def _locate_outputs(self) -> None:
        pdos = assigned_pdos(self.sdo, 0x1C12)
        self._controlword_bits = None
        self._target_bits = None
        for pdo in pdos:
            for entry in pdo.get("entries", []):
                if entry.get("index") == 0x6040 and entry.get("subindex") == 0:
                    self._controlword_bits = entry["bit_offset"]
                if entry.get("index") == 0x607A and entry.get("subindex") == 0:
                    self._target_bits = entry["bit_offset"]
        for bits in (self._controlword_bits, self._target_bits):
            if bits is not None and bits % 8 != 0:
                raise SafetyBlocked(["RPDO object is not byte-aligned; refusing to drive process data"])

    def _publish(self) -> None:
        output = self.slave.output
        if not output:
            return
        buf = bytearray(output)
        if self._controlword_bits is not None:
            start = self._controlword_bits // 8
            buf[start:start + 2] = pack_u16(self._controlword)
        if self._target_bits is not None:
            start = self._target_bits // 8
            buf[start:start + 4] = pack_i32(self._target)
        self.slave.output = bytes(buf)

    def _pump(self, cycles: int = 5) -> int:
        wkc = 0
        for _ in range(cycles):
            self._publish()
            self.master.send_processdata()
            wkc = self.master.receive_processdata(2_000)
            if self._stop:
                raise KeyboardInterrupt
        return wkc

    def _enter_op(self) -> None:
        """PySOEM sizes this slave from truncated PDO reads. Do not request SAFEOP.

        The verified 15/28-byte image is installed by build/lc_e_csp_hold, with
        the controlword held at 0. This method never enables the servo.
        """
        raise SafetyBlocked([
            "PySOEM cannot install the verified 15/28-byte LC10E image; "
            "disabled OP is build/lc_e_csp_hold and this path does not enable the servo",
        ])

    def _command(self, controlword: int, expected: str, timeout_s: float = config.STATE_TIMEOUT_S) -> dict:
        self._controlword = controlword
        self.sdo.slave.sdo_write(0x6040, 0, pack_u16(controlword))
        holder = {"last": {}}

        def arrived():
            if self._mapped:
                self._pump(1)
            holder["last"] = self.read_status()
            last = holder["last"]
            if last["error"]:
                raise RuntimeError(f"error 0x{last['error']:04X} after controlword 0x{controlword:04X}")
            if last["cia402"] in ("Fault", "Fault Reaction Active"):
                raise RuntimeError(f"fault after controlword 0x{controlword:04X}")
            return last["cia402"] == expected

        try:
            wait_until(arrived, timeout_s, time.sleep, time.monotonic, interval_s=0.02)
        except TimeoutError as exc:
            raise TimeoutError(
                f"timeout waiting for {expected}, last={holder['last'].get('cia402')}"
            ) from exc
        return holder["last"]

    def enable(self) -> dict:
        status = self._require_ready_for_power()
        self._enter_op()
        try:
            self._command(0x0006, READY)
            self._command(0x0007, SWITCHED_ON)
            self._enabled = True
            return self._command(0x000F, OPERATION_ENABLED)
        except Exception:
            self._enabled = True
            self.disable()
            raise

    def disable(self) -> dict:
        if self.sdo is None:
            self._enabled = False
            return {}
        try:
            shutdown, disable_voltage = fault_shutdown_words()
            self._controlword = shutdown
            self.sdo.slave.sdo_write(0x6040, 0, pack_u16(shutdown))
            if self._mapped:
                self._pump(10)
            self._controlword = disable_voltage
            self.sdo.slave.sdo_write(0x6040, 0, pack_u16(disable_voltage))
            if self._mapped:
                self._pump(10)
        finally:
            self._enabled = False
        return self.read_status()

    def stop(self) -> dict:
        return self.disable()

    def _profile(self, counts: int, speed_rpm: float) -> None:
        scaling = self.read_scaling()
        velocity, acceleration = profile_units(speed_rpm, scaling.command_units_per_rev)
        self.sdo.slave.sdo_write(0x6081, 0, pack_u32(velocity))
        self.sdo.slave.sdo_write(0x6083, 0, pack_u32(acceleration))
        self.sdo.slave.sdo_write(0x6084, 0, pack_u32(acceleration))
        self._target = counts
        self.sdo.slave.sdo_write(0x607A, 0, pack_i32(counts))

    def _move_to(self, target: int, speed_rpm: float) -> dict:
        scaling = self.read_scaling()
        current = self.read_position()
        if current is None or not scaling.known or not scaling.command_units_per_rev:
            raise SafetyBlocked(move_blockers(0.0, speed_rpm, scaling))
        delta_degrees = counts_to_degrees(target - current, scaling.command_units_per_rev)
        blockers = move_blockers(delta_degrees, speed_rpm, scaling)
        if blockers:
            raise SafetyBlocked(blockers)
        status = self.read_status()
        if status["cia402"] != OPERATION_ENABLED:
            self.enable()
        self.sdo.slave.sdo_write(0x6060, 0, pack_i8(PP_MODE))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            mode, _err = self.sdo.optional(self.sdo.read_i8, 0x6061, 0)
            if mode == PP_MODE:
                break
            time.sleep(0.02)
        else:
            raise TimeoutError("0x6061 did not become profile position (1)")
        self._profile(target, speed_rpm)
        setpoint, immediate = absolute_setpoint_words()
        self._command(setpoint, OPERATION_ENABLED)
        self._command(immediate, OPERATION_ENABLED)
        start = self.read_position()
        scaling = self.read_scaling()
        limit_counts = abs(degrees_to_counts(5.0, scaling.command_units_per_rev))
        speed_limit = int(scaling.command_units_per_rev / 5.0)
        deadline = time.monotonic() + config.MOVE_TIMEOUT_S
        last = {}
        misses = 0
        expected_wkc = 3
        while time.monotonic() < deadline:
            wkc = self._pump(1) if self._mapped else expected_wkc
            misses, tripped = note_wkc(wkc, expected_wkc, misses)
            if tripped:
                self.disable()
                raise RuntimeError("process-data working counter missed three cycles")
            last = self.read_status()
            if last["error"] or last["cia402"] in ("Fault", "Fault Reaction Active"):
                self.disable()
                raise RuntimeError(f"fault during move: {last}")
            if (self.slave.state & 0x0F) != pysoem.OP_STATE:
                self.disable()
                raise RuntimeError("slave left OP")
            if last["velocity"] is not None and abs(last["velocity"]) > speed_limit:
                self.disable()
                raise RuntimeError("velocity exceeded 0.2 rev/s")
            if start is not None and last["position"] is not None and abs(last["position"] - start) > limit_counts:
                self.disable()
                raise RuntimeError("position error exceeded 5 degrees from the start of this move")
            if (
                last["statusword"] is not None
                and target_reached(last["statusword"])
                and last["position"] is not None
                and abs(last["position"] - target) <= 2
                and (last["velocity"] is None or abs(last["velocity"]) < 5)
            ):
                return last
            time.sleep(0.05)
        self.disable()
        raise TimeoutError("move timed out")

    def move_absolute(self, degrees: float, speed_rpm: float = config.MAX_SPEED_RPM) -> dict:
        scaling = self.read_scaling()
        current = self.read_position()
        if current is None or not scaling.known or not scaling.command_units_per_rev:
            raise SafetyBlocked(move_blockers(degrees, speed_rpm, scaling))
        target = degrees_to_counts(degrees, scaling.command_units_per_rev)
        delta_degrees = counts_to_degrees(target - current, scaling.command_units_per_rev)
        blockers = move_blockers(delta_degrees, speed_rpm, scaling)
        if blockers:
            raise SafetyBlocked(blockers)
        return self._move_to(target, speed_rpm)

    def move_relative(self, degrees: float, speed_rpm: float = config.MAX_SPEED_RPM) -> dict:
        scaling = self.read_scaling()
        blockers = move_blockers(degrees, speed_rpm, scaling)
        if blockers:
            raise SafetyBlocked(blockers)
        current = self.read_position()
        if current is None:
            raise SafetyBlocked(["position feedback missing"])
        delta = degrees_to_counts(degrees, scaling.command_units_per_rev)
        # Absolute target. Relative bit 6 is not used.
        return self._move_to(current + delta, speed_rpm)


def _print_status(status: dict) -> None:
    scaling = status["scaling"]
    print(f"EtherCAT state: {status['ethercat_state']}")
    print(f"CiA402 state: {status['cia402']}")
    error = status["error"]
    print(f"error code: {error if error is not None else 'unreadable'}")
    print(f"current mode: {status['mode']}")
    print(f"position: {status['position']}")
    print(f"velocity: {status['velocity']}")
    print(f"torque: {status['torque']}")
    if scaling.known:
        print(f"command units per revolution: {scaling.command_units_per_rev}")
    else:
        print(f"POSITION_SCALING_UNKNOWN: {scaling.reason}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="LC-E read-only status, or a latched enable/disable")
    parser.add_argument("command", choices=("status", "enable", "disable"))
    args = parser.parse_args(argv)
    nic.ensure_root()
    servo = LichuanServo()
    try:
        servo.connect()
        if args.command == "status":
            _print_status(servo.read_status())
            return 0
        if args.command == "enable":
            try:
                status = servo.enable()
            except SafetyBlocked as exc:
                print("ENABLE REFUSED")
                for item in exc.blockers:
                    print(f"  - {item}")
                return 3
            _print_status(status)
            return 0
        status = servo.disable()
        _print_status(status)
        return 0
    finally:
        servo.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
