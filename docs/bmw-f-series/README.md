# BMW F-series (F11) notes for the BMW_F11 vehicle class

These are the notes that the `BMW_F11` code (`src/vehicles/BMW_F11.cpp`, `include/vehicles/BMW_F11.h`), the host test in `test/bmw_f11_pico/`, and pull requests #276 and #277 refer to. They come from the car tests of the Pico FlexRay DDE/EGS emulator.

**Test vehicle:** 2011 BMW F11 520d, VIN `WBAMX12080V788571`. (Over OBD the DDE gave `WBAMX12080C788571`. The 11th character differs, and we have not checked which one is right.)

**Immobiliser:** the ZV never asks the car for an immobiliser exchange, and in testing the car never asked for one, so nothing is needed.

## The ZV ↔ Pico link

| File | What it is |
|---|---|
| [`pico_zv_private_can.md`](pico_zv_private_can.md) | The split between the ZV and the Pico (draft 2): the ZV sends every BMW PT-CAN frame, and the Pico only sends its private frames. §5 is the PT-CAN timing the ZV copies, and §6 is the private link 0x304-0x30C. A note at the top lists where the ZV code differs. |
| [`pico_zv_private_can.dbc`](pico_zv_private_can.dbc) | The private frames 0x304-0x30C as a DBC (draft 2 plus the v1z.1 bit 31). |
| [`pico_dde_emu_v1z1_readme.md`](pico_dde_emu_v1z1_readme.md) | README of the Pico firmware v1z.1 that the ZV is matched to: what the Pico does next to the ZV, wiring, and its car-test list. |

## What the DDE receives (decoded on the Pico, shown by the ZV only)

| File | What it is |
|---|---|
| [`dde_dsc_torque_rx.md`](dde_dsc_torque_rx.md) | DSC torque and intervention requests to the DDE (FlexRay only). The ZV shows these as F11DscAct / F11DscNm. |
| [`dde_cruise_rx.md`](dde_cruise_rx.md) | Cruise control signals to the DDE. The ZV shows these as F11CrsAct / F11CrsNm. |

## Gearbox

| File | What it is |
|---|---|
| [`egs_gear_report_tx.md`](egs_gear_report_tx.md) | What the real EGS sends when the gear changes (0x197 lever in, 0x3FD / 0x39A / 0x3F9 gear out). The gear logic is built from it. |

## Car test notes

Times are seconds from the start of each capture unless a note says otherwise.

| Note | Run | Pico firmware | In one line |
|---|---|---|---|
| [`f11_emu_run4.md`](f11_emu_run4.md) | 4 | v1d | Real DDE vs no DDE on PT-CAN: the DDE's own PT-CAN frames and timing (the code's "run 4" 0x397 blips come from here). |
| [`f11_emu_run6.md`](f11_emu_run6.md) | 6 | v1e | First full FlexRay + PT-CAN emulation. Clean dash, P shown, MIL off, normal sleep. |
| [`f11_emu_run7_v1f.md`](f11_emu_run7_v1f.md) | 7 | v1f | 20 ms PT-CAN frames at the full rate; KL15 on/off like run 6. |
| [`f11_emu_run7_brake.md`](f11_emu_run7_brake.md) | 7 | v1f | Brake signal and brake switches, FlexRay compared with run 6. |
| [`f11_emu_run8_v1g.md`](f11_emu_run8_v1g.md) | 8 | v1g.2 | First fake engine start through CAS (ST_KL D). |
| [`f11_emu_run8_start_can.md`](f11_emu_run8_start_can.md) | 8 | v1g.2 | The start sequence on PT-CAN. |
| [`f11_emu_run8_start_fr.md`](f11_emu_run8_start_fr.md) | 8 | v1g.2 | The start on FlexRay: timeline, end of crank, chassis reactions. |
| [`f11_emu_run9_v1h1.md`](f11_emu_run9_v1h1.md) | 9 | v1h.1 | First gear shifts (P/R/D). |
| [`f11_emu_run9_gears_can.md`](f11_emu_run9_gears_can.md) | 9 | v1h.1 | Gear selection on PT-CAN. |
| [`f11_emu_run9_fr.md`](f11_emu_run9_fr.md) | 9 | v1h.1 | The FlexRay view of run 9. |
| [`f11_emu_run10_v1h2.md`](f11_emu_run10_v1h2.md) | 10a/10b | v1h.2 | OEM-like: no fault codes, brake gating works. |
| [`f11_emu_run10_gears_can.md`](f11_emu_run10_gears_can.md) | 10a/10b | v1h.2 | Gear and PT-CAN check of run 10. |
| [`f11_emu_run10_fr.md`](f11_emu_run10_fr.md) | 10a/10b | v1h.2 | FlexRay + PT-CAN cross-check of run 10 against runs 8 and 9. |

## Notes for readers

- The ZV code follows Pico firmware v1h.3 for PT-CAN timing and v1z.1 for the private link. Those are pico-flexray source files (for example `ptcan_emu_core.c`, `dde_engine.c`, `dde_gear.c`, `zv_link.c`). They are not in this repository.
- The notes mention captures, CSV files, scripts and other notes under `/workspace/...` or `analysis/...`. Those are not in this repository.
- These notes are copies of the originals, trimmed for this repository.
