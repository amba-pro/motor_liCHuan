"""PDO mapping decode. A record is accepted only when it is a real 32-bit mapping entry."""

from __future__ import annotations

from sdo import SdoClient, sdo_error_text

# Objects that may be driven while the servo stays disabled.
# 0x6040 is forced to 0. 0x607A is copied from the current position.
# Speed and torque targets are forced to 0. Anything else is refused,
# including probe and maximum-speed objects, because writing them changes
# configuration and P0C.13 can store 6000h writes.
SAFE_OUTPUTS = {
    0x6040: 16,
    0x607A: 32,
    0x60FF: 32,
    0x6071: 16,
}

SM_TYPE = {
    0: "unused",
    1: "mailbox out (master to slave)",
    2: "mailbox in (slave to master)",
    3: "process outputs (RxPDO)",
    4: "process inputs (TxPDO)",
}


def decode_mapping_record(data: bytes) -> dict | None:
    """Decode one PDO mapping entry. Returns None unless the entry is 4 bytes.

    The low byte is the mapped bit length. It is not assumed to be 32.
    """
    if len(data) != 4:
        return None
    value = int.from_bytes(data, "little")
    if value == 0:
        return {"empty": True, "bits": 0}
    bits = value & 0xFF
    if bits == 0 or bits == 0xFF:
        return None
    return {
        "index": (value >> 16) & 0xFFFF,
        "subindex": (value >> 8) & 0xFF,
        "bits": bits,
        "empty": False,
    }


def soem_assumed_bits(raw_entries: list[bytes]) -> int:
    """Bit count SOEM accumulates: the low byte of each subentry, zero-padded to 4 bytes.

    A short subentry is not a mapping record, but SOEM still adds that low byte.
    """
    total = 0
    for raw in raw_entries:
        padded = (raw + b"\x00\x00\x00\x00")[:4]
        total += padded[0] if padded[0] != 0xFF else 0xFF
    return total


def payload_bits(records: list[dict]) -> int:
    return sum(record["bits"] for record in records if not record.get("empty"))


def assess_raw_pdo(raw_entries: list[bytes]) -> dict:
    """Decide whether this PDO can be exchanged without inventing output bytes."""
    records = []
    ambiguous = False
    for sub, raw in enumerate(raw_entries, start=1):
        decoded = decode_mapping_record(raw)
        if decoded is None:
            ambiguous = True
            records.append({
                "sub": sub,
                "length": len(raw),
                "raw": raw.hex(),
                "ambiguous": True,
            })
            continue
        decoded["sub"] = sub
        records.append(decoded)
    mapped = [record for record in records if record.get("bits")]
    bits = payload_bits(records) if not ambiguous else None
    soem_bits = soem_assumed_bits(raw_entries)
    reason = ""
    if ambiguous:
        reason = (
            "one or more PDO subentries are not 4-byte mapping records; "
            f"SOEM would still size the SyncManager from their low bytes "
            f"({soem_bits} bits, {(soem_bits + 7) // 8} bytes)"
        )
    else:
        unknown = [
            record for record in mapped
            if SAFE_OUTPUTS.get(record["index"]) != record["bits"]
        ]
        if unknown:
            ambiguous = True
            shown = ", ".join(f"0x{item['index']:04X}:{item['bits']} bits" for item in unknown)
            reason = f"PDO contains objects that are not safe to drive while disabled: {shown}"
    return {
        "records": records,
        "ambiguous": ambiguous,
        "safe": not ambiguous and bool(mapped),
        "payload_bits": bits,
        "payload_bytes": None if bits is None else (bits + 7) // 8,
        "soem_bits": soem_bits,
        "soem_bytes": (soem_bits + 7) // 8,
        "reason": reason,
    }


def build_disabled_output(records: list[dict], actual_position: int) -> bytes:
    """Build a disabled RxPDO image. Raises ValueError if the map is not safe."""
    raw_standin = []
    for record in records:
        if record.get("empty"):
            raw_standin.append(b"\x00\x00\x00\x00")
        elif record.get("ambiguous") or "index" not in record:
            raise ValueError("ambiguous PDO")
        else:
            value = (record["index"] << 16) | (record["subindex"] << 8) | record["bits"]
            raw_standin.append(value.to_bytes(4, "little"))
    report = assess_raw_pdo(raw_standin)
    if not report["safe"]:
        raise ValueError(report["reason"] or "PDO is not safe")
    buffer = bytearray(report["payload_bytes"])
    bit = 0
    for record in records:
        if record.get("empty"):
            continue
        if record["index"] == 0x6040:
            encoded = (0).to_bytes(2, "little")
        elif record["index"] == 0x607A:
            encoded = int(actual_position).to_bytes(4, "little", signed=True)
        elif record["index"] in (0x60FF, 0x6071):
            encoded = (0).to_bytes(record["bits"] // 8, "little")
        else:
            raise ValueError(f"refusing to encode 0x{record['index']:04X}")
        if bit % 8 != 0:
            raise ValueError("mapped object is not byte aligned")
        start = bit // 8
        buffer[start:start + len(encoded)] = encoded
        bit += record["bits"]
    return bytes(buffer)


def _mapping(client: SdoClient, index: int, sub: int) -> dict:
    try:
        raw = bytes(client.slave.sdo_read(index, sub, 0))
    except Exception as exc:
        return {"sub": sub, "error": sdo_error_text(exc)}
    decoded = decode_mapping_record(raw)
    if decoded is None:
        return {"sub": sub, "raw": raw.hex(), "length": len(raw), "ambiguous": True}
    decoded["sub"] = sub
    return decoded


def pdo_entries(client: SdoClient, pdo_index: int) -> list[dict]:
    n_entries, n_err = client.optional(client.read_u8, pdo_index, 0)
    if n_entries is None:
        return [{"error": n_err or "unreadable"}]
    entries = []
    bit_offset = 0
    for sub in range(1, int(n_entries) + 1):
        entry = _mapping(client, pdo_index, sub)
        if entry.get("bits"):
            entry["bit_offset"] = bit_offset
            bit_offset += entry["bits"]
        entries.append(entry)
    return entries


def assigned_pdos(client: SdoClient, assign_index: int) -> list[dict]:
    count, err = client.optional(client.read_u8, assign_index, 0)
    if count is None:
        return [{"error": err or "assignment unreadable"}]
    pdos = []
    for slot in range(1, int(count) + 1):
        pdo_index, slot_err = client.optional(client.read_u16, assign_index, slot)
        if pdo_index is None:
            pdos.append({"slot": slot, "error": slot_err})
            continue
        pdos.append({"slot": slot, "pdo": pdo_index, "entries": pdo_entries(client, pdo_index)})
    return pdos


def read_raw_entries(slave, pdo_index: int) -> tuple[int, list[bytes]]:
    count = bytes(slave.sdo_read(pdo_index, 0, 1))[0]
    return count, [bytes(slave.sdo_read(pdo_index, sub, 0)) for sub in range(1, count + 1)]
