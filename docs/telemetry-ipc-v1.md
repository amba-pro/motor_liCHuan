# Local telemetry protocol v1 (work in progress)

This PR adds a **read-only Qt client**, not the production EtherCAT telemetry publisher.

Transport: a local Unix socket exposed as Qt `QLocalServer` under name `lichuan-telemetry-v1`.
Format: newline-delimited UTF-8 JSON, <=4096 bytes per line.

One example message:

```json
{"schema":1,"source":"lc_e_csp_hold","position_counts":-4224994,"velocity_counts_s":0,"torque_raw":0,"following_counts":0,"statusword":545,"error_code":0,"wkc":3,"op":true,"enabled":false}
```

The example is **illustrative, not measured live data**.

The receiver rejects missing, invalid, unknown-source or out-of-range fields and marks data stale after 500 ms without updates. Stale data are not silently re-labelled as current. All real commands from Qt stay blocked. The existing demo is not replaced or renamed as hardware.

## Producer requirements before live integration is declared complete

- The one and only C++/SOEM master owns `enp37s0`.
- Sampling the PDO inputs occurs within the existing 1 ms loop. Network/IPC serialization and socket writes must occur **outside** the real-time loop, through a bounded nonblocking snapshot queue.
- Use verified TxPDO 0x1B02 field offsets; the existing PDO does not include velocity 0x606C, so velocity must come from a separately validated measurement, not fabricated from torque or an assumed type. If unavailable, extend the schema to allow explicitly unavailable velocity rather than send a guessed value.
- Producer authenticates/limits local socket peer, rejects multiple publishers, never accepts control/motion messages on telemetry socket.
- Set restrictive Unix socket permissions and confirm no unauthenticated user can publish fake “real” values; `source` in JSON alone does not establish authenticity.
- Enforce exclusive access to EtherCAT NIC with the repository’s lock mechanism.
- Do not relax the 250 us deadline or bypass hardware safety interlocks.
- Maintain stable client behaviour on disconnect, stale/partial frames, crashes and restart.
- No motor enable or physical movement while implementing and testing telemetry.

## Local verification

```sh
cmake -S . -B build -DSOEM_ROOT=/home/cv/projects/SOEM
cmake --build build -j
ctest --test-dir build --output-on-failure
.venv/bin/python -m unittest discover -s tests
QT_QPA_PLATFORM=offscreen build/ui/lc_motor_control --self-check
```

A green Qt test **does not** mean live SOEM producer is implemented or connected.
