"""Known LC10E PDO layouts and the disabled image built from one exact match.

Vendor 0x00000766, product 0x00000402, revision 0x00000204 identifies LC10E V1.04.
The V1.04 entry list is the IgH PDO listing of that identity. The 20250705 manual
describes a different 0x1702 / 0x1B02. A layout is usable only when the live
object-dictionary widths match exactly one of these lists.
"""

from __future__ import annotations

from cia402 import enables_operation

# index, subindex, bit length
Map = list[tuple[int, int, int]]

# IgH PDO listing for identity 0x00000766 / 0x00000402 / 0x00000204.
V104_RX_1702: Map = [
    (0x6040, 0, 16),
    (0x607A, 0, 32),
    (0x60B8, 0, 16),
    (0x6071, 0, 16),
    (0x607F, 0, 32),
    (0x6060, 0, 8),
]
V104_TX_1B02: Map = [
    (0x603F, 0, 16),
    (0x6041, 0, 16),
    (0x6064, 0, 32),
    (0x6077, 0, 16),
    (0x60F4, 0, 32),
    (0x60B9, 0, 16),
    (0x60BA, 0, 32),
    (0x60BC, 0, 32),
    (0x60FD, 0, 32),
]

# LC-E manual 20250705 section 6.4.2. Different object order and widths.
MANUAL_RX_1702: Map = [
    (0x6040, 0, 16),
    (0x607A, 0, 32),
    (0x60FF, 0, 32),
    (0x6071, 0, 16),
    (0x6060, 0, 8),
    (0x60B8, 0, 16),
    (0x607F, 0, 32),
]
MANUAL_TX_1B02: Map = [
    (0x603F, 0, 16),
    (0x6041, 0, 16),
    (0x6064, 0, 32),
    (0x6077, 0, 16),
    (0x6061, 0, 8),
    (0x60B9, 0, 16),
    (0x60BA, 0, 32),
    (0x60BC, 0, 32),
    (0x60FD, 0, 32),
]

RX_CANDIDATES = {
    "LC10E V1.04 0x1702": V104_RX_1702,
    "LC-E manual 20250705 0x1702": MANUAL_RX_1702,
}
TX_CANDIDATES = {
    "LC10E V1.04 0x1B02": V104_TX_1B02,
    "LC-E manual 20250705 0x1B02": MANUAL_TX_1B02,
}

# Written every cycle. Controlword stays 0. A velocity target stays 0.
_FORCED = {
    0x6040: 0,
    0x60FF: 0,
}
# Copied from the current SDO so the cycle does not change a limit, mode, or torque target.
_ECHO = {0x60B8, 0x6071, 0x607F, 0x6060}


def widths(mapping: Map) -> list[int]:
    return [bits for _index, _sub, bits in mapping]


def payload_bytes(mapping: Map) -> int:
    bits = sum(widths(mapping))
    if bits % 8 != 0:
        raise ValueError("PDO is not byte aligned")
    return bits // 8


def match_widths(live_widths: list[int], candidates: dict[str, Map]) -> str | None:
    """Return the candidate name only when exactly one layout has these widths."""
    hits = [name for name, mapping in candidates.items() if widths(mapping) == list(live_widths)]
    if len(hits) != 1:
        return None
    return hits[0]


def build_disabled_image(mapping: Map, actual_position: int, echoes: dict[int, int]) -> bytes:
    """Disabled Rx image. Controlword is 0. Position is echoed. Limits are echoed.

    Raises ValueError if an object has no safe encoding. 0x607F, 0x60B8 and
    0x6060 must be supplied unchanged; they are not forced to zero.
    """
    buffer = bytearray(payload_bytes(mapping))
    bit = 0
    for index, subindex, bits in mapping:
        if subindex != 0 or bits % 8 != 0 or bit % 8 != 0:
            raise ValueError(f"unsupported mapping 0x{index:04X}:{subindex:02X}/{bits}")
        size = bits // 8
        if index == 0x607A:
            encoded = int(actual_position).to_bytes(size, "little", signed=True)
        elif index in _FORCED:
            encoded = int(_FORCED[index]).to_bytes(size, "little", signed=False)
        elif index in _ECHO:
            if index not in echoes:
                raise ValueError(f"missing unchanged value for 0x{index:04X}")
            encoded = int(echoes[index]).to_bytes(size, "little", signed=True)
        else:
            raise ValueError(f"no disabled encoding for 0x{index:04X}")
        buffer[bit // 8:bit // 8 + size] = encoded
        bit += bits
    image = bytes(buffer)
    if enables_operation(int.from_bytes(image[:2], "little")):
        raise ValueError("image would enable the drive")
    if image[:2] != b"\x00\x00":
        raise ValueError("controlword is not 0")
    return image


def can_use_soem_buffer(verified_bytes: int, soem_bytes: int) -> bool:
    """SOEM sizes this slave from truncated mapping reads. Shrinking is possible.

    Growing is not: the process image is already allocated.
    """
    return verified_bytes > 0 and verified_bytes <= soem_bytes
