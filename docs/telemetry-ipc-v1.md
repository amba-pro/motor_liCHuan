# Local telemetry protocol v1

The Qt client is read-only. `lc_e_csp_hold --telemetry` is the only publisher. It samples the verified TxPDO inside the existing 1 ms loop and writes JSON on another thread. `velocity_counts_s` is JSON null because 0x606C is not in that PDO.

Transport: a local Unix socket exposed as Qt `QLocalServer` under name `lichuan-telemetry-v1`.
Format: newline-delimited UTF-8 JSON, <=4096 bytes per line.

One example message:

```json
{"schema":1,"source":"lc_e_csp_hold","position_counts":5028064,"velocity_counts_s":null,"torque_raw":0,"following_counts":0,"statusword":592,"error_code":0,"wkc":3,"op":true,"enabled":false}
```

The example matches one disabled-servo OP capture (position 5028064 counts, statusword 0x0250). It is not a continuous reading.

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

A green Qt test does not by itself show that the physical drive was in OP. Live numbers require `lc_e_csp_hold --telemetry` and a fresh frame.
