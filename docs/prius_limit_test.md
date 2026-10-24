# Prius Gen3 power limit test

A test build for Prius Gen3 / Auris inverters (`Inverter` = `Prius_Gen3`), to find out what limits power to about 24 kW. It lets you change four bytes of the frame the VCU sends to the inverter from the web interface, without recompiling.

This build is for testing only. It is not meant to be merged.

## Firmware to use

| | |
|---|---|
| Version shown on the web interface | **`2.41.0TA.PT1`** |
| Branch | `toyota-prius-limit-test` |
| Firmware commit | `e4b284c` |
| CI build | [run 40106240896](https://github.com/damienmaguire/Stm32-vcu/actions/runs/40106240896) (passed) |
| Firmware binary (`stm32_vcu.bin`) | [Stm32-vcu firmware binary](https://github.com/damienmaguire/Stm32-vcu/actions/runs/40106240896/artifacts/12564643239) |
| Firmware hex (`stm32_vcu.hex`) | [Stm32-vcu firmware hex](https://github.com/damienmaguire/Stm32-vcu/actions/runs/40106240896/artifacts/12564274771) |

The downloads need a GitHub login. Each one is a zip with the file inside. After flashing, check that the web interface shows version `2.41.0TA.PT1`.

Later commits on this branch that only change this file start new CI runs with the same firmware. Use the build linked above.

The build is based on the open PR damienmaguire/Stm32-vcu#272 (shared Toyota sync serial fixes), so it also has those changes.

## What the test params do

Four test params in **Gearbox Control**. They are never saved and go back to their defaults at every power-up. The defaults are the values the VCU has always sent, so the build drives like before until you change one.

| Param | Frame bytes | Default | OEM value (Auris, driving) | What it is |
|---|---|---|---|---|
| `PriusRegenLim` | 76-77 | 9568 | about -2000 | battery regen (charge) limit |
| `PriusDischLim` | 78-79 | 1106 | about 1680 | battery discharge limit |
| `PriusByte86` | 86 | 137 | 202 | ready flags |
| `PriusByte88` | 88 | 137 | 239 | possibly a speed limit |

The VCU has always sent start-up values in these bytes, not the values the car uses while driving. The regen limit even has the wrong sign. Units aren't known yet; finding them is part of the test.

Leave everything else as it is. Byte 73 still tells the inverter that the boost converter is bypassed, which matches how ZombieVerter Prius inverters are wired.

## What to log

From the web interface, at full pedal, with power plotted against MG2 rpm (`speed`):

`udc`, `idc` (shunt), `INVudc`, `speed`, `torque`, and power (`udc` × `idc`).

## Steps

Change one thing at a time. Values go back to their defaults after a power cycle, so set them again after every restart.

1. **Baseline.** Leave all four at their defaults. Confirm the ~24 kW ceiling and note the rpm where power stops rising.
2. **OEM-like limits.** `PriusRegenLim` = -2000, `PriusDischLim` = 1660.
3. **Higher discharge limit.** `PriusDischLim` = 2212. If the ceiling moves in step with this value, this byte pair is the limiter, and the step tells us its unit.
4. **Ready flags.** Only if steps 2 and 3 change nothing: `PriusByte86` = 202.
5. **Byte 88.** Then `PriusByte88` = 239.

At 350-400 V pack voltage the motor should manage roughly 32-37 kW before running out of voltage, so there is room to see a change.

## What to report

For each step: pack voltage, peak power, the rpm where power levels off, and any errors or odd behaviour (`InvStat`, `DMA_ConsecFail`). Plots or log files are the most useful.
