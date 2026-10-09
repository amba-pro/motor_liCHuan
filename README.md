# Lichuan LC10E EtherCAT controller

Native tools for one Lichuan LC10E servo on a dedicated Ethernet port. The realtime master is C++ and [SOEM](https://github.com/OpenEtherCATsociety/SOEM) 1.4.0. A Qt 6 Widgets application provides a Russian operator panel and a local demonstration. The panel does not open the EtherCAT socket.

This tree does not enable or move the motor by itself. Every physical-safety latch in `python/config.py` stays false until a person sets it after a real check.

## Hardware this code was written for

| Item | Value |
| --- | --- |
| Drive | Lichuan LC10E-100W, vendor `0x00000766`, product `0x00000402`, revision `0x00000204` |
| Motor | LCMT-01SLR23ZB-40M00330B, 100 W, 3000 rpm, 23-bit absolute encoder |
| Command units | 8,388,608 per revolution (`1°` = 23,302 counts) |
| PDO in use | Rx `0x1702` 15 bytes, Tx `0x1B02` 28 bytes, expected working counter 3 |
| Mode under test | CSP (`0x6060` = 8). Profile Position bits are not confirmed |
| Cycle | 1 ms. A wake later than 250 µs is a missed deadline and is not accepted |

The commissioning machine used interface `enp37s0`. That name is only the default in the tools. Pass the port that is reserved for EtherCAT and is not the route used by the operating system.

## What is not in this repository

These files are third-party and are not redistributed here:

- SOEM 1.4.0. Clone [OpenEtherCATsociety/SOEM](https://github.com/OpenEtherCATsociety/SOEM) at tag `v1.4.0` and build it. Point `SOEM_ROOT` at that tree, or place it next to this project as `../SOEM`.
- The LC10E ESI XML (the V1.04 device description). Obtain it from Lichuan. `python/esi_lc10e.py` records only the object index, subindex, and bit width of the maps this project checks. It is not a copy of the XML.
- The LC-E manual (20250705). Its `0x1702` / `0x1B02` layout differs from the V1.04 map and is not the one used on the bus.

## Ubuntu dependencies

Tested on Ubuntu 24.04.

```sh
sudo apt-get install -y build-essential cmake python3 python3-venv \
  qt6-base-dev qt6-charts-dev
```

CMake 3.16 or newer is required. The Python tools use PySOEM 1.1.13:

```sh
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

PySOEM cannot install the verified 15/28-byte process image. Operational EtherCAT frames are sent by the C++ master. Python refuses to request SAFEOP for that map.

## Layout

| Path | Role |
| --- | --- |
| `src/` | SOEM master, CiA402 helpers, disabled OP, CSP hold, CSP enable-and-hold |
| `python/` | Read-only identification, safety latches, CSP checks, trajectory preview |
| `tests/` | Python tests for safety, PDO maps, CSP, and the 250 µs / 30 s timing rule |
| `ui/` | Qt 6 Widgets panel, charts, demo plant, and Qt tests |
| `scripts/run_diag.sh` | Read-only SDO diagnostic. It does not write the controlword |

## Build

```sh
cmake -S . -B build -DSOEM_ROOT=/path/to/SOEM
cmake --build build -j
```

If `../SOEM/soem/ethercat.h` exists, `-DSOEM_ROOT` can be omitted. SOEM's static library must already be at `$SOEM_ROOT/build/libsoem.a`.

Executables:

- `build/lc_e_diag` — read-only SDO diagnostic
- `build/lc_e_control` — enable path, still gated by an explicit flag and the safety latches
- `build/lc_e_op_disabled` — cyclic OP with controlword 0
- `build/lc_e_csp_hold` — disabled 1 ms benchmark
- `build/lc_e_csp_enable` — stationary CSP enable hold; requires `--confirm HOLD`
- `build/ui/lc_motor_control` — Qt panel, built when Qt 6 Widgets and Charts are found

## Qt panel

```sh
QT_QPA_PLATFORM=xcb ./build/ui/lc_motor_control
```

On a Wayland session the platform has to be `xcb` or the window can open on a fake screen. An offscreen check that resizes the window and saves page captures:

```sh
QT_QPA_PLATFORM=offscreen ./build/ui/lc_motor_control --self-check
```

Demo mode is the only mode the window runs. It integrates a local trajectory on its own thread and labels every value as demonstration data. Buttons for real servo enable and real motion do not open a socket and do not construct a second master. Closing the window stops that demo thread. It does not replace a hardware emergency stop.

## Tests

```sh
.venv/bin/python -m unittest discover -s tests
./build/ui/lc_ui_logic_test
QT_QPA_PLATFORM=offscreen ./build/ui/lc_ui_window_test
```

## Safety limits that stay in force

- `MOTION_ARMED`, `BRAKE_CIRCUIT_VERIFIED`, `ESTOP_AVAILABLE`, `MOTOR_SECURED`, `SHAFT_CLEAR`, `MECHANICALLY_SAFE`, `AXIS_LIMITS_KNOWN`, `PP_BITS_CONFIRMED`, and `CSP_TIMING_ACCEPTED` are false in `python/config.py`. Do not set them from a script.
- Command envelope, once motion is ever armed: 1 degree, 5 rpm, 20 rpm/s, torque demand at most 20% of rated.
- The 250 µs wake-lateness limit on the 1 ms cycle is not relaxed. A disabled 30 s run is accepted only with 30,000 cycles, a clean working counter, no fault, and no sample later than 250 µs.
- The external 24 V brake is not on CN2. Disabling the servo does not apply it. Loss of communication does not apply it. Software Stop is not a hardware emergency stop.
- The tools do not write EEPROM, PDO assignment `0x1C12` / `0x1C13`, or the profile objects `0x607F`, `0x6081`, `0x6083`, and `0x6084`. They do not auto-reset faults.

## Current status

Identity, the V1.04 15/28-byte map, a disabled OP cycle, and the encoder scale have been exercised on the drive. CSP mode 8 can be displayed while the controlword stays 0. One stationary enable hold reached Operation Enabled and then shut down because a single 1 ms deadline was missed. A later 30 s disabled benchmark was not accepted: two cycles were late, the worst by 1.78 ms, while the frame itself stayed near 30 µs. The 1 degree move has not been run. CSV velocity mode is not offered as an operational command. The Qt application is a working demo and does not connect to the drive.
