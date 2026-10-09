"""Little-endian CoE reads that treat an unsupported object as data, not a crash."""

from __future__ import annotations

import pysoem

from cia402 import unpack_i8, unpack_i16, unpack_i32, unpack_u16, unpack_u32, unpack_u8


class UnsupportedSdo(Exception):
    def __init__(self, index: int, subindex: int, detail: str) -> None:
        self.index = index
        self.subindex = subindex
        self.detail = detail
        super().__init__(f"0x{index:04X}:{subindex:02X} unsupported: {detail}")


def sdo_error_text(exc: BaseException) -> str:
    abort = getattr(exc, "abort_code", None)
    desc = getattr(exc, "desc", None)
    if abort is None:
        return str(exc)
    text = f"abort 0x{abort:08X}"
    if desc:
        text += f" {desc}"
    return text


class SdoClient:
    def __init__(self, slave) -> None:
        self.slave = slave

    def read_typed(self, index: int, subindex: int, size: int, unpack):
        try:
            data = self.slave.sdo_read(index, subindex, size)
        except (pysoem.SdoError, pysoem.MailboxError, pysoem.PacketError, pysoem.WkcError) as exc:
            raise UnsupportedSdo(index, subindex, sdo_error_text(exc)) from exc
        try:
            return unpack(bytes(data))
        except ValueError as exc:
            raise UnsupportedSdo(index, subindex, str(exc)) from exc

    def read_u8(self, index: int, subindex: int = 0):
        return self.read_typed(index, subindex, 1, unpack_u8)

    def read_i8(self, index: int, subindex: int = 0):
        return self.read_typed(index, subindex, 1, unpack_i8)

    def read_u16(self, index: int, subindex: int = 0):
        return self.read_typed(index, subindex, 2, unpack_u16)

    def read_i16(self, index: int, subindex: int = 0):
        return self.read_typed(index, subindex, 2, unpack_i16)

    def read_u32(self, index: int, subindex: int = 0):
        return self.read_typed(index, subindex, 4, unpack_u32)

    def read_i32(self, index: int, subindex: int = 0):
        return self.read_typed(index, subindex, 4, unpack_i32)

    def read_visible_string(self, index: int, subindex: int = 0, size: int = 64) -> str:
        try:
            data = self.slave.sdo_read(index, subindex, size)
        except (pysoem.SdoError, pysoem.MailboxError, pysoem.PacketError, pysoem.WkcError) as exc:
            raise UnsupportedSdo(index, subindex, sdo_error_text(exc)) from exc
        return bytes(data).split(b"\x00", 1)[0].decode("utf-8", "replace")

    def optional(self, reader, index: int, subindex: int = 0):
        try:
            return reader(index, subindex), None
        except UnsupportedSdo as exc:
            return None, str(exc)
