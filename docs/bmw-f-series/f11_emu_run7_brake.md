# F11 run 7 (emulator v1f, no DDE, EGS fuse out): brake signal, brake switches, FlexRay vs run 6

Written 2026-11-01 (Europe/Dublin, GMT). Scripts: `analysis/emu_run7_brake_scripts/` (`run_all.sh` re-runs everything in ~20 s). Table: `analysis/f11_emu_run7_brake.csv`.
Labels: **confirmed** = measured, plus an independent known reference. **likely** = strong evidence but the meaning is inferred. **speculative/guess** = hypothesis.
Byte convention: I give **0-based byte index** and **bN (1-based, SavvyCAN data[N], Sam's convention)**. Bit positions are Intel/LE start bits within the payload. L = plen+1.
Times: seconds from the first FR record of run 7 (Sam's CSV times are ~0.01 s later). KL15 on/off comes from PT-CAN 0x12F d2 low nibble (ST_KL).

## 0. Data and caveats
- Input: `/workspace/f11-emu-run7/replay/f11_emu7_v1f.csv` (+`.can.csv`). This is Sam's conversion (10:10 GMT). I made my own replay with the v1f `fr_ts_logger.py --replay … --can` while it was pending, and the two are **byte-identical** (`cmp`). Totals equal the console `done:` line: FR1 1,262,238, CAN 152,797, bus2 109 IDs.
- What the logger records: **one FR channel (FR1 = vehicle bus 0, DSC side) and one CAN channel (PT-CAN, Ian's bus 3)**. **No bus 1, no bus 2/PT-CAN2.** Emulator TX shows on FR1 because the emulator sits on that bus. No own CAN TX rows (Sam's check: FAIL).
- **Logger drops**: seq_missing 187,933 / seq_gaps 39,489. The loss depends on slot position: it ramps from about 12 % at static slot 55 to about 50 % at slot 70, and dynamic slots ≥0x61 are mostly lost. So **0x3F (slot 63) lost about 30 %** of its frames (1,916 gaps >30 ms), while 0x2B lost 96. Peak *values* are consistent, but rise times measured on 0x3F can be late by 20–80 ms. 0x2B / 0x0EF torque is used as the timing reference.
- Timeline:
  - wake 1 at 0 s with no KL15 (ST_KL 4→5→6); the bus sleeps 76.6–115.5;
  - wake 2: KL15 on 119.652, presses 141–163, KL15 off 204.702; the bus sleeps at 257.4 (OFF+52.7, same as run 6);
  - wake 3 at 270.8 (trigger unknown), still awake at the end (306.0).

## 1. Brake pressure / travel candidates (presses P1 gentle, P2 firmer, P3 full)
Rest = value with the brake released. CRC (b1) and alive (b2 low nibble) bytes are excluded, as are 0x3F b9/b10 (CRC/alive of the second PDU in 0x3F).

| rank | signal | frame | position | rest | P1 / P2 / P3 raw | scaled (guess) | label |
|---|---|---|---|---|---|---|---|
| **1** | **pressure "Y"** | **FR 0x3F, plen 16 (L17), base cycle 1, rep 4 (20 ms)** | 0-based byte 13 bits 4–7 + byte 14 (b14 high nibble + b15); Intel start bit 108, 12 bit unsigned | 0 | **214 / 557 / 1310** at 142.81 / 150.33 / 159.61 | 0.0625 bar/LSB → **13.4 / 34.8 / 81.9 bar**; alternative 0.1 bar → 21.4 / 55.7 / 131 bar | **likely** master-cylinder pressure; scaling **guess** |
| 2 | travel-like "X" | same frame | 0-based byte 12 + byte 13 bits 0–3 (b13 + b14 low nibble); Intel start bit 96, 12 bit | 0 | 303 / 465 / 739 at 142.81 / 150.33 / 159.61 | 0.1 mm/LSB → 30 / 47 / 74 mm | **speculative** pedal travel or driver demand |
| 3 | brake torque (known) | FR 0x2B c%4=3, b3–b4 (bytes 2–3) u16 LE −32000; = **PT-CAN 0x0EF d2–d3** | 1 Nm/bit | 32000 (0 Nm) | −1134 / −2952 / −6959 Nm at 143.04 / 150.34 / 159.58 | – | **confirmed** (`dde_brake_rx.md`); torque B (b6–b7) −1136 / −2958 / −6953 |
| 3 | torque copy | FR 0x3F c%4=1 bytes 10–11 (b11–b12) u16 LE −32000 | | 32000 | −1132 / −2945 / −6948 | Nm | likely |
| 4 | per-wheel torques | FR 0x2C c%4=3, u16 LE −32000: bytes 2–3, 4–5 (b3–6) and 6–7, 8–9 (b7–10) | | 32000 | rear? −185 / −484 / −1141 each; front? −381 / −991 / −2339 each | Nm | likely per-wheel (2·rear + 2·front = torque A within 100 Nm); front/rear assignment speculative |
| 5 | unknown | FR 0x2C c%4=3 bytes 12–13 (b13–b14) s16 LE | | −2 | −680 / −1771 / −4094 (clips at −4096) | ≈ −3.18·Y | speculative |
| 6 | brake level 0–6 | PT-CAN 0x25D d4 bits 4–7 = FR 0x70 c%4=3 (L9) byte 4 (b5) high nibble, 100 ms | | 0 | 2 / 3 / 6 | class | speculative (coarse brake-intensity or decel level; levels overlap in Y) |

- **PT-CAN carries no pressure or travel signal.** The only continuous brake value on PT-CAN is the 0x0EF torque copy plus the 0x25D nibble. Bus 2/PT-CAN2 was not logged.
- Correlations (20 ms grid, KL15 window):
  - Y vs torque A −0.9998 (fit torque = −5.31·Y, residual 65 Nm);
  - Y vs X 0.961;
  - Y vs 0x25D nibble 0.990;
  - X vs torque −0.961;
  - per-wheel vs torque 1.000.
  - X is **concave** against Y: it moves first and saturates (P3: Y 38 → X 164; Y 608 → X 486; Y 1310 → X 739). That is how travel behaves against pressure, which is why I suggest X = travel (still speculative; an F-series pedal-travel sensor is not established).
- Why 0.0625 bar is the better scaling guess (**speculative**):
  - torque/Y = 5.31 Nm/LSB = 85 Nm/bar total at 1/16 bar, which gives about 28.6 Nm/bar per front wheel and 13.9 per rear wheel;
  - my rough caliper estimate (assumed piston sizes, not measured) is ~27 front and ~14 rear;
  - at 0.1 bar the implied figure is 53 Nm/bar;
  - Sam's own `brake_peaks_run7.csv` also uses 1/16 bar (b15 integer);
  - an unloaded full-force press of 80–130 bar is plausible either way.
- The pressure-threshold bit (0x173 d7 bit 2 / FR 0x2F b8 bit 2) switches at Y ≈ 66–92 on / 67–161 off. That is ≈ 5 bar at 1/16 bar (≈ 450 Nm torque), consistent with the old "~7 bar" note on the pseudo-scale.

## 2. Brake switch timing (evidence)
| source | P1 on / off | P2 on / off | P3 on / off |
|---|---|---|---|
| PT-CAN 0x12F d6 (30↔33) | 141.527 / 144.806 | 146.912 / 151.497 | 155.552 / 163.152 |
| FR 0x74 c%4=2 b7 (byte 6) &3 | 141.533 / 144.813 | 146.913 / 151.553 | 155.553 / 163.152 |
| PT-CAN 0x173 d7 &3 | 141.548 / 144.827 | 146.927 / 151.517 | 155.567 / 163.167 |
| (FR 0x2F odd b8 &3, source of 0x173) | 141.556 / 144.826 | 146.926 / 151.516 | 155.566 / 163.166 |
| 0x173 d7 bit 2 (pressure threshold) | 142.007 / 144.627 | 147.127 / 151.307 | 155.837 / 162.767 |

- **First non-zero torque** (pressure rise): 141.616 / 146.956 / 155.596.
- **Lag from switch on to the pressure rise:**
  - 0x12F: +90 / +44 / +44 ms
  - 0x74: +83 / +43 / +43 ms
  - 0x173: +68 / +29 / +29 ms
- **The switch leads the pressure every time.** Order: 0x12F ≈ 0x74 (switch) first, then 0x173 15–20 ms later.
- **Last non-zero torque** 144.856 / 151.576 / 163.176 is +24…+79 ms after the 0x12F switch off; P2 at 0x74 is the exception (it clears at 151.553).
- The 0x74 copy only flips on cycle%4=2. The c%4=0 copy of 0x74 does **not** follow the brake (confirmed).

## 3. FlexRay vs run 6 (v1e)
**No new or missing emulator slots**; the set is the same 0x26/28/29/3D (L17), 0x5C, E6, E8 L9/L5, FB L5/L7/L9, 0x10A, 0x142, 0x68.
- **Content (per-byte modal values and value sets, pre/KL15/post-off phases):**
  - **no differences** in 0x26/28/29/3D/0x142/E8/0x10A;
  - E6 b1/b2 and FB L5 b1 differ only in CRC or counter values (not significant).
- **Gating:**
  - E8 L5 first at ON+0.21 (v1e +0.24, real +0.19);
  - 0x10A first at ON+0.18 (v1e +0.21, real +0.16), last at OFF−0.87 (1 s grid);
  - both stop at OFF as before.
- After each wake, v1f emulator frames start within ~60 ms.
- Rate differences in the "only run 7" vehicle slots (0xFC/0xFE/0x105/0x113–0x115 etc.) are **not new IDs**. These are vehicle ECUs: after a cold wake they first appear at ~34 s in both runs, and after run 7's re-wake within ~1 s.
- Console: lock_losses 3 (one per wake), late 31 (v1e 28), echo miss 1.
  - skip E6=4, FB7=24, FB5=1, FB9=1, E8L9=3 (v1e: E6=1, E8L9=2).
  - **skip2 0x142 = 1 (v1e 26)**.
  - Gaps in the captured frames cannot be separated from logger drops in this run.

**Flagged items** (real-DDE references: KL15-only captures 4log_ignonoff and mitm_v2_transparent_100r, plus engine-run captures; `ref_postoff.py`):

| item | real DDE | v1e run 6 | v1f run 7 | status |
|---|---|---|---|---|
| FB L9 b3–4 → ff 7f after OFF | extra off-schedule frame at OFF+0.23 / +0.25 s (KL15-only); +0.51…+1.52 s after engine runs | OFF+5.07 (frame at +0.07 still 04 00) | **OFF+0.86**, on the normal 5 s grid (−4.14, +0.86, +5.86) | **improved, likely fixed for the value**. Still no event frame, so latency depends on 5 s phase (0–5 s) |
| 0x29 c3 b8 → c0 after OFF | OFF+0.02 / +0.04 (KL15-only), +0.28…+1.31 (engine) | OFF+0.07 | OFF+0.07; back to 80 at re-wake | matches the real DDE (present in both, correct) |
| 0x142 b2 "counter" | b2 = state 00 → 01 (wake+2.5 s) → 03 (ON+1.57); never steps at b1 wraps | same, 03 at ON+1.59 | same, 03 at ON+1.59. Resets to 00 / b1 restarts at 1 on each wake, as the real DDE does at wake | **not a counter in any run, and correct**. b1 counts 0..254 (never 255) +1 per 20 ms; 0 non-+1 steps at 20 ms spacing. The b2 changes at "wraps" (2) are the wake resets |

- **Inconsistency in the references:** in the ddecan (run 4a) real capture, neither FB L9 ff 7f nor 0x29 c0 appears within 52 s after OFF. The other two real KL15-only captures show both. The cause is unknown.

## 4. Open points
- The scaling of Y (bar) and the meaning of X need a gauge or a known-pressure reference (for example a DSC diagnostic read of the pressure sensor during a press).
- Front/rear assignment of the 0x2C per-wheel fields needs a check.
- Sending FB L9 post-off as an event (~+0.2 s) would match the KL15-only real DDE more closely.
