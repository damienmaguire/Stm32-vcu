# EGS gear reporting for the F11 emulator (DDE+EGS role): 0x197 lever in, gear frames out

Written 2026-11-01 (Europe/Dublin). This is v1, built from the one real F11 shift capture plus the F30 EGS firmware/A2L. Owned hardware, documentation only.

**Labels:** [CONFIRMED] = seen directly in a capture or in firmware/A2L. [ANALYSIS] = derived from that evidence. [HYPOTHESIS] = not tested.

**Companion files**
- `egs_gear_report_tx.csv`: one row per frame and field, with values for P/R/N/D.
- `egs_gear_report_scripts/`:
  - `timeline.py`, `shift_detail.py`, `phases.py`, `fr_gear.py`, `sample_ids.py`, `scan_lever.py`, `e2e.py`;
  - saved outputs in `*_r10md10m.txt`.

**Sources**
- Capture: `egs-logs/F11_4log_r10md10m.csv`. `8hp45-logs/` holds byte-identical copies (same md5).
- Damien's code: `gws_tmp/F30_Lever.cpp`.
- F30 GA8P75 firmware: `8hp-firmware/firmware_flash.bin` (base 0) and `ecu.a2l`.
- Earlier notes: `egs_gws_lever_can.md`, `egs_8hp_map.md`, `f11_full_emu_ptcan.md`, `f11_emu4_flexray_emulate_list.csv`, `8hp45-dump/CANTCU_CAN_MAP.md`.
- Public: openinverter wiki "BMW F-Series Gear Lever"; projectgus GWS posts, parts 1 and 3.

---

## 0. The capture: format, buses, what happens

- **Format** [CONFIRMED]: a SavvyCAN 4-bus CSV with columns `Time Stamp,ID,Extended,Dir,Bus,LEN,D1..`.
  - The timestamp is µs within the current second (ms resolution) and wraps every 1 s. `flexray-re-notes/ptcan/ptload.py` unwraps it per bus.
  - All times below are seconds from file start. They are capture-relative, not wall-clock.
- **Bus numbering** [CONFIRMED]:
  - Bus 0 = FlexRay vehicle side. Bus 1 = FlexRay DDE side (0x26/0x28/0xE6/0x142 …).
  - **Bus 2 = PT-CAN2.** It has 48 IDs, including 0x1AF/0x0A0/0x0B0/0x0F8, and no 0x173/0x1B3/0x202.
  - **Bus 3 = PT-CAN.** It has 135 IDs, including 0x173 brake, 0x1B3, 0x202 and diagnostics 0x618/0x6F0.
  - 0x12F and 0x197 are on **both** buses, with identical payloads.
  - In the emulator's own single-bus `.can.csv` logs (runs 4–8), "bus 2" is PT-CAN. Do not mix the two numberings.
- **Other captures checked** (`scan_lever.py`): `F11_4log_3krev`, `F11_4log_ignonoff` and the `f11-emu-run4..8` CAN logs.
  - All of them show 0x197 idle only (`x 0F C0`).
  - `f11-run2`, `f11-run3` and `f11-ts-run*` are FlexRay-only (no 0x197).
  - **r10md10m is the only real shift capture.** [CONFIRMED]
- **What happens in r10md10m** [CONFIRMED]. The file is 114 s long, so "r10md10m" means roughly 10 m in R and 10 m in D. Every lever action was made with the brake pressed.

| t (s) | Event |
|---|---|
| 6.44 | Brake on: 0x173 d7 = D7, 0x12F d6 = 0x33 |
| 6.4–7.3 | Engine start |
| 7.441 | 0x3FD → P |
| 16.42 | **P → R** (lever up, two notches) |
| 17.6–32 | Reversing |
| 33.08 | **R → D** (lever down, two notches) |
| 34–42 | Driving forward |
| 42.58 | **P button in D → P** |
| 45.6 | Park pawl confirmed (0x39A B4 = E2) |
| 80.06 | 0x3FD → 00 (off) |

  Never exercised in this capture: N held as a selection, the side/M gate (M+/M−), double P presses, a shift without brake, or the unlock button on its own.

---

## (a) 0x197 ST_GWS lever decode (F11)

DLC 4, from the GWS to the EGS. It goes out on **both** PT-CAN and PT-CAN2 within the same ms. The interval alternates 22/38 ms (mean 30 ms) and the EGS timeout is 90 ms. [CONFIRMED capture + F30 record]

| Byte.bits | Field (EGS A2L name) | F11 values | Label |
|---|---|---|---|
| B0 | CRC8 E2E: poly 0x1D, init 0, over `[0x3F, 0x00, B1, B2, B3]`, **data ID 0x003F**. Same as Gus's xorout K = 0x53 over B1..B3 | 7582/7582 frames pass | [CONFIRMED fw E2E table 0x46D08 + capture] |
| B1.0-3 | alive counter 0..14 (never 15) | | [CONFIRMED] |
| B1.4-7 | `iv_PosAbsS1/S2Raw` | 0 idle. **2** at Up1 and at Down1. **1** at Up2 (it stays 1 on the way back through Up1). **3** at Down2 (it stays 3 on the way back through Down1) | values [CONFIRMED]. Meaning (latched depth/zone) [HYPOTHESIS]. The emulator can ignore it |
| B2.4-7 | `iv_PosRelS1/S2Raw`, the relative lever move | **0 idle, 1 Up1, 2 Up2, 3 Down1, 4 Down2**. 5 side+up (M−?), 6 side+down (M+?), 7 side gate centre (from Damien/Gus, **not seen on F11**) | 0–4 [CONFIRMED F11]. 5–7 [ANALYSIS, F30/Gus] |
| B2.0-3 | low nibble: B2.0-1 `GWS_S_Button`, B2.2-3 `DriveLogic` | **F11 = 0xF in every frame** (S button = 3, DriveLogic = 3). Damien's F30 code expects 0xE | [CONFIRMED capture + fw getters] |
| B3 | P button: B3.0-1 PS, B3.2-3 P1, B3.4-5 P2, B3.6-7 LowMode = 3 | **0xC0 released, 0xD5 pressed** | [CONFIRMED, P press at 42.58 s lasted 200 ms] |

**F11 decode = Damien's table with `(B2 & 0xF0)`.**

| B2 (F11) | Damien F30 | Meaning |
|---|---|---|
| 0x0F | 0x0E | idle / centre |
| 0x1F | 0x1E | up (forward) one notch |
| 0x2F | 0x2E | up two notches ("up-double") |
| 0x3F | 0x3E | down (back) one notch |
| 0x4F | 0x4E | down two notches ("down-double") |
| 0x5F / 0x6F / 0x7F | 0x5E / 0x6E / 0x7E | side+up / side+down / side centre. Not seen on F11 [HYPOTHESIS] |

- **Quirk** [CONFIRMED]: Damien's `switch (bytes[2])` exact-matches `0x?E`, so on an F11 lever every case falls to `default` and nothing is ever decoded.
  - Fix: `switch (bytes[2] & 0xF0)`. [ANALYSIS]
  - Also check the E2E and alive counter (Damien skips both), and treat the frame as lost after 90 ms without a valid one.
- **"Unlock button"** [CONFIRMED absent]: there is **no unlock bit in 0x197**.
  - The EGS A2L lists every 0x197 input (`GearLeverIn.*`, `SportButtonIn.*`, `LowModeDriveIn.*`) and has no unlock signal.
  - Gus reports that pressing Unlock "didn't change anything in this message".
  - The **GWS gates the lever itself**, using the gear it is shown in 0x3FD. In P (and for R) it does not report Up/Down moves unless Unlock is held (Gus, openinverter).
  - So in r10md10m, the fact that Up1/Up2 were reported from P means Unlock was held. [ANALYSIS]
  - Emulator consequence: you cannot and need not check Unlock. You **must** keep 0x3FD showing the true gear, because the GWS gating depends on it. [ANALYSIS]
- **"Double" means two notches, not a double tap.** Up2 or Down2 is the lever pushed through to the second detent, and B2 passes 1 → 2 (or 3 → 4) on the way. [CONFIRMED capture]
  - A double-press of the P button was not captured. Damien needs 3 consecutive D5 frames (`PrkCnt > 2`), about 90 ms. [CONFIRMED source]

### How the EGS turns lever moves into a gear
F30 calibration `D_Tab_RelToAbsConversion` is at flash 0x21804A. It is a 7×4 u8 map: rows are `eLeverRel` (FFF, FF, F, NA, B, BB, BBB) and columns are the current position (D, N, R, P). Output enum `ePosition`: 5 = D, 6 = N, 7 = R, 0 = no request. [CONFIRMED A2L + flash bytes]

| Lever (0x197 B2 hi) | from D | from N | from R | from P |
|---|---|---|---|---|
| Up1 (F, 1) | **N** | **R** | R | **N** |
| Up2 (FF, 2) | **R** | R | R | **R** |
| Down1 (B, 3) | D | **D** | **N** | **D** |
| Down2 (BB, 4) | D | D | **D** | D |
| idle (NA, 0) | – | – | – | – |
| P button (B3 = D5) | → P (separate path: `P1/P2` → P request) | → P | → P | stays P [HYPOTHESIS] |

- In the table, "R" means R is requested. Whether R/D/P is actually allowed is decided afterwards by the shift-lock and plausibility logic.
- The table's interpretation is [ANALYSIS]. The capture confirms these cells: P+Up1→N, P+Up2→R, R+Down2→D, and D+P-button→P.
- The EGS re-evaluates on the deepest position reached (`RelToAbs.MaxLeverpos` "maximum lever position until lever is back in NA"). It acts while the lever is still deflected; it does not wait for the release. [CONFIRMED A2L names + capture: N shows at Up1 and R at Up2, both before release]
- **Debounce / latency** [CONFIRMED capture, interpretation ANALYSIS]:
  - The 0x3FD change comes **40 ms (±1 ms)** after the lever frame, at Up1→N, Up2→R and Down2→D alike.
  - A position held for only ~21 ms is skipped. Down1 (R→N) was present on both buses together only from 33.078 to 33.099, and no N was displayed.
  - Emulator rule: act on a lever code only after it has been stable for ≥ 2 consecutive valid frames (or ≥ 40 ms) on the bus(es) you read.
- **Brake requirement:**
  - The firmware has shift-lock sub-modules `ShiftLock.Shiftlock_Brake / _Engine / _RemoteStart / _CAN`.
  - It has CC messages `ID_ShiftlockBrakeWarning` ("Push brake to insert gear") and `ID_ShiftWithoutBrakeWarning`.
  - `SWI_Shiftlock_Disable` (0x207E24) = 0x00000012; which bit disables which sub-module is unknown.
  - `D_v_Shiftlock_Uth` (raw 80) is the speed above which the shift-lock is dropped.
  - Labels: [CONFIRMED A2L/flash]; bit mapping [HYPOTHESIS].
  - In the capture, **every** shift was made with the brake pressed: 0x173 d7 = 0xD7 (`d7 & 3 == 3`) and 0x12F d6 = 0x33. On release, 0x12F d6 = 0x30 and 0x173 d7 = C0/D0. [CONFIRMED]
  - Rule to emulate: leaving P, and P/N → R/D at standstill, need the brake (`0x173 d7&3==3`, or `0x12F d6 == 0x33` as a second source). Ignore the request and keep the gear (optionally blink, see b) when the brake is not pressed. [HYPOTHESIS: no brake-off shift was captured]
- **Unlock requirement:** enforced by the GWS, not the EGS (see above). [ANALYSIS]
- **P button:** acts from R/N/D. The EGS first goes to an internal "neutral" state (0x39A gear code 1) and shows P only after the pawl is engaged (about 0.9 s, see b). [CONFIRMED capture]
  - Damien's "P button in P → N" is **not** EGS behaviour. The EGS leaves P only by lever (Up1→N, Up2→R, Down→D). [ANALYSIS from map]
- **Damien vs EGS differences** [ANALYSIS]. Damien's state machine:
  - ignores N+Up1→R, N+Down1→D, D+Up2→R, P+Up1→N, P+Up2→R and P+Down→D;
  - has R+Down2 → N, where the EGS gives D;
  - starts in N, where the EGS starts in the actual state (P);
  - forces P when not in RUN, where the EGS sends 00 when off.
  - For Sam's build, implement the map above instead.

---

## (b) Everything the EGS sends that changes with gear

**FlexRay** [CONFIRMED, `egs_8hp_map.md` §1]: **the EGS is not on FlexRay.** Gear reaches FlexRay only inside DDE PDUs (FR 0xE6, 0x28 c3, 0x142 on bus 1). Because the emulator also plays the DDE, those are listed too.

**IDs 0x0BA / 0x1A0 / 0x0B5 do not exist in any F11 capture** (not on bus 2 or bus 3). [CONFIRMED] They are not 8HP45/F11 frames. [ANALYSIS]

### Per-gear table (values from r10md10m unless labelled)

| ID | Bus | Cycle | E2E | Byte/bit | P | R | N | D | Label |
|---|---|---|---|---|---|---|---|---|---|
| **0x3FD** DT_DISP_GRDT (EGS) | PT-CAN + PT-CAN2, same ms | 100 ms + **event frame on change** | ID 0x0039 (K 0x70), B1 lo counter 0..14, hi 0 | B2 (bits 5-7 position) | **0x20** | **0x40** | **0x60** | **0x80** | [CONFIRMED]; N seen only as a 60 ms transit |
| | | | | B2 bit3 (0x08) blink | 0 | 0 | 0 | 0 | never set in capture. "+0x08 = flash" [ANALYSIS, Gus] |
| | | | | B2 bits 0-2 | 0 | 0 | 0 | 0 | 0x81 = D with M/S allowed (Gus/Damien) [HYPOTHESIS for F11] |
| | | | | B3, B4 | FC FF | FC FF | FC FF | FC FF | constant [CONFIRMED] |
| | | | | off state | 0x00 (pre-start and from 80.06 s) | | | | [CONFIRMED] |
| **0x39A** (EGS) | both | 1000 ms + event | none | B3 gear code | **03** | **02** | 01? | **05** | P/R/D [CONFIRMED]. 01 seen for 0.86 s during D→P while 0x3FD still showed D; "1 = N/transit" [HYPOTHESIS] |
| | | | | B4 (bits 0-1 P-lock; upper bits 0xE0) | **E2** (pawl in) | E0 | E0? | E0 | [CONFIRMED]. **E1** for 100 ms on leaving P. E3 at power-up (SNA). E0→E2 only 2.1 s after P display (pawl confirmed) |
| | | | | B1 | 3B/3C | | | | oil temp °C = raw − 40, not gear [CONFIRMED prior note] |
| **0x3F9** (DDE, relays EGS) | both | 1000 ms + event (2–40 ms after 0x39A) | **ID 0x0037**, B1 lo counter | B3 bits 4-5 (bits 28-29) = 0x39A B4 bits 0-1 | **2** (B3 = 0x26/0x64) | 0 (0x06) | 0? | 0 (0x06) | [CONFIRMED]. Also 1 during the 100 ms P-release. Drives 0x12F field42 (CAS) |
| | | | | B6 low nibble gear code (= 0x39A B3) | **3** | **2** | 1? | **5** | [CONFIRMED], 1 during D→P transit |
| **0x396** (EGS) | both | 1000 ms | none | B0..B1 | fd ff | fd ff | – | fd ff | constant [CONFIRMED] |
| **0x518** EGS NM | both | ~640 ms | none | – | no gear dependence | | | | [CONFIRMED] |
| **0x1AF** (EGS) | **PT-CAN2 only** | 20 ms | ID 0x0006 (K 0x43) | B7 engaged range | **01** | **06** (ramps 02→06 over 0.36 s, 1.1–1.5 s after lever) | 01? | **0B / 09** (06→0E→0B in 40 ms) | [CONFIRMED]. 00 before start |
| | | | | B2 | **FF** | 05–08 | FF? | 07–0C | [CONFIRMED]. FF = no range engaged [ANALYSIS] |
| | | | | B3..4 turbine rpm+2000, B5..6 output rpm+2000 | | | | | load/speed [CONFIRMED prior] |
| **0x0A0** (EGS TX table) | PT-CAN2 only | 10 ms | ID 0x00A0 | B2..B5 | slow drift + torque-like 16-bit (B4..5) | | | | no clean gear code [ANALYSIS] |
| **0x0B0** (EGS TX table) | PT-CAN2 only | 10 ms | ID 0x0001 | B4..5 16-bit | 0x803F | ~0x821F–0x831F | ? | ~0x82BF–0x831F | torque/load-like, settles back to 0x803F in P [ANALYSIS] |
| 0x0F8 (EGS TX table) | PT-CAN2 only | 20 ms | none (alive B0 lo) | | | | | | rpm-like, no gear code [ANALYSIS] |
| 0x0A6 / 0x0A7 / 0x2B1 | PT-CAN2 only | | | | small load drift | | | | not in the EGS TX table → not EGS [ANALYSIS] |
| FR **0xE6** b3..b8 (DDE) | FR bus 1 | 1 s + event | FR E2E | same layout as 0x3F9 B2..B7: b4 bits 4-5 P-lock, b7 low nibble gear code | 26/03 | 06/02 | ? | 06/05 | [CONFIRMED]. The 100 ms P-lock = 1 state was not copied |
| FR **0x28 c3** b11 / b14 (DDE) | FR bus 1 | | | b11 / b14 | **22 / FE** | **25 / F4** | ? | **24 / F4** | [CONFIRMED]. D→P transit 22/FC |
| FR **0x142** b12 (DDE) | FR bus 1 | | | b12 | 80 | 82→84 (94 brief) | ? | 88/98/90 → 80 | gear+brake mix [ANALYSIS]. Prior emu list: 80/84/80 |

The only GWS/KOMBI-visible selector/display frame is **0x3FD**. No separate selector-LED frame exists: the GWS LEDs follow 0x3FD (Gus). 0x202 (dimming) is body/gateway, not EGS. [CONFIRMED/ANALYSIS, earlier note]

### Real-shift timelines (bus 3 = PT-CAN unless noted) [CONFIRMED]

**P → R** (brake pressed since 6.46 s, Unlock implied):

| t (s) | Event | Δ from lever |
|---|---|---|
| 16.421 | 0x197 B2 = 1F (Up1), B1hi = 2, both buses | 0 |
| 16.441 | 0x39A B4 E2→**E1** (P-lock releasing), B3 still 03 | +20 ms |
| 16.443 | 0x3F9 B3 0x26→0x16 (bits 28-29: 2→1) | +22 ms |
| 16.461 | **0x3FD B2 = 0x60 (N)**, event frame, both buses | **+40 ms** |
| 16.482 | 0x197 B2 = 2F (Up2), B1hi = 1 | 0 (second event) |
| 16.521 | **0x3FD B2 = 0x40 (R)** (+ a grid frame 2 ms later) | **+39 ms after Up2**, N lasted 60 ms |
| 16.542 | 0x39A B3 = **02**, B4 = **E0** | +60 ms after Up2 |
| 16.544 | 0x3F9 B3 = 0x06, B6 = 02 | +62 ms |
| 16.537 | FR 0xE6 b4/b7 = 06/02 (cross-bus alignment ±10 ms) | |
| 16.744 / 16.804 | lever back 1F → 0F | no change |
| 17.58–17.94 | 0x1AF B7 01→02→…→06 (R clutch filling/engaged) | **+1.16…1.52 s** |

**R → D** (brake D7 since 32.78 s):

| t (s) | Event |
|---|---|
| 33.060 (bus 2) / 33.078 (bus 3) | Down1 (3F), only ~21 ms on both buses → **no N shown** |
| 33.099 | Down2 (4F) |
| 33.139 | **0x3FD = 0x80 (D)**, **+40 ms** |
| 33.141 | 0x39A B3 = 05 |
| 33.178 | 0x3F9 B6 = 05 (+37 ms after 0x39A) |
| 33.139–33.179 | 0x1AF B7 06→0E→0B (D engaged in ≈ 40–80 ms; it was already in a forward-ready state) |
| 33.34 / 33.38 | lever back to 0F |

**D → P (P button)** (brake D7 since 42.50 s):

| t (s) | Event |
|---|---|
| 42.580 | 0x197 B3 = D5 for 200 ms (≈ 7 frames) |
| 42.599 / 42.639 | 0x1AF B2 = FF, then B7 = 01 (+20/+59 ms) |
| 42.601 | 0x39A B3 = **01** (+21 ms); 0x3F9 B6 = 01 at 42.622 |
| 42.630 | FR 0x28 b11/b14 = 22/FC |
| **0x3FD stays 0x80 (D) for 900 ms** | — |
| 43.461 | 0x39A B3 = 03 |
| 43.480 | **0x3FD = 0x20 (P)**, **+900 ms after the button** |
| 43.490 | FR 0x28 = 22/FE |
| 45.08–45.27 | brake released |
| 45.604 | 0x39A B4 **E0→E2** (pawl confirmed), 0x3F9 B3 → 0x26 (bits 28-29 = 2) at 45.606: **+3.0 s after the button, +2.1 s after the P display** |

**Blink:** no blink state (B2 | 0x08) and no intermediate value other than the 60 ms N was ever sent. [CONFIRMED]

**Event behaviour** [CONFIRMED]: 0x3FD, 0x39A and 0x3F9 each send an immediate extra frame on change, then carry on with their normal grid. The 0x3FD counter increments on every frame, event frames included.

---

## Emulator implementation checklist (DDE+EGS on PT-CAN)

1. **Read 0x197** on PT-CAN (and on PT-CAN2 if it is wired). Steps 1–3 are [ANALYSIS, F11 capture + F30 fw]:
   - Verify E2E ID 0x003F and a changing alive counter.
   - Decode with `B2 & 0xF0` (F11 low nibble is F). The P button is `B3 == 0xD5`.
   - Accept a code only when it is stable for ≥ 2 frames (~40 ms). If no valid frame arrives for 90 ms, hold the gear and flag a fault.
2. **Gear state machine** = `D_Tab_RelToAbsConversion` (table in a), gated by:
   - the brake (`0x173 d7&3==3`, or `0x12F d6==0x33`) for leaving P and for P/N→R/D [HYPOTHESIS that the EGS refuses without it];
   - a speed check before P or R (`D_v_Shiftlock_Uth` / park-lock abuse threshold) [HYPOTHESIS].
   - P is entered only by the P button (≥ 3 frames of D5), or optionally automatically on KL15 off.
   - Do not do Damien's "P button in P → N".
3. **Act on the deepest notch reached.** Up1 then Up2 from P gives N then R; that is fine and matches the EGS. Ignore the return to idle.
4. **0x3FD** (minimum frame), sent on PT-CAN **and** PT-CAN2:
   - Layout `[CRC(0x0039), ctr 0..14, pos, 0xFC, 0xFF]`. Use pos 0x20/0x40/0x60/0x80, and 0x00 when the system is off.
   - Send every 100 ms, plus an immediate frame on change. Start the counter at 1 after wake.
   - Examples with ctr = 1: P `8c 01 20 fc ff`, R `dc 01 40 fc ff`, N `ec 01 60 fc ff`, D `7c 01 80 fc ff`. P is checked against the capture; R/N/D are computed with the checked CRC.
   - Labels: [CONFIRMED layout/E2E]; values from `e2e.py`.
   - Display the gear the **EV has actually accepted**. Update it ~40 ms after the lever, and for P only once your "park" is confirmed (the real EGS waits about 0.9 s).
5. **0x39A** (both buses, 1 s + event, no E2E):
   - Layout `00 <oiltemp+40> 00 <gear> <E0|plock> 00`, with gear 3/2/(1)/5 and B4 E2 in P, E0 otherwise. Use E3 for the first ~0.1 s after wake.
   - Fault-free byte 2 = 0x00. Bits 2-3/6-7 = 1 was seen in the no-DDE fault case (`f11_full_emu_ptcan.md` Q4).
   - Labels: [CONFIRMED values] / [HYPOTHESIS for N = 1].
6. **0x3F9** (your DDE role, ID 0x0037):
   - Copy the P-lock into bits 28-29: **2 in P, 0 otherwise**. Optionally 1 for ~100 ms when leaving P.
   - Copy the gear code into the B6 low nibble.
   - Send an event frame within ~20 ms of a change.
   - If bits 28-29 ≠ 2 in P, CAS/FEM 0x12F field42 goes 0 and the terminal walk at OFF fails (`f11_full_emu_ptcan.md` §b). [CONFIRMED chain]
7. **0x396** `fd ff` and **0x518** NM: constant, not gear-dependent. [CONFIRMED]
8. **FlexRay DDE PDUs:**
   - FR 0xE6 b4 bits 4-5 and b7 low nibble: same as 0x3F9.
   - FR 0x28 c3 b11/b14: 22/FE P, 25/F4 R, 24/F4 D.
   - FR 0x142 b12: 80 P, 84 R, 80 D.
   - Keep the FR 0x26 turbine field at d0 07 (0 rpm), or a live motor speed.
   - Labels: [CONFIRMED P/R/D values]; N [HYPOTHESIS].
9. **PT-CAN2-only frames** (0x1AF, 0x0A0, 0x0B0, 0x0F8): their only consumer was the DDE, which is gone. Omit them unless something real still listens on PT-CAN2. If you do send 0x1AF, use B7 01/06/0B and B2 FF in P. [ANALYSIS]

**Gotchas**
- F11 low nibble 0xF: mask with 0xF0. [CONFIRMED]
- The GWS enforces Unlock from what 0x3FD shows. A wrong 0x3FD gives wrong lever gating and LEDs. Never run Damien's `F30_Lever` 0x3FD alongside another 0x3FD source. [ANALYSIS]
- The GWS reads 0x3FD from the EGS on **both** PT-CAN and PT-CAN2 (GWS DTC text: E09400/E09410). If PT-CAN2 stays wired to the GWS, send 0x3FD there too, or expect a GWS DTC. [HYPOTHESIS]
- No blink and no M/S mode were captured. Use 0x08 (blink) only to signal a refused shift [HYPOTHESIS].
- The P display lags the request. If your park is instant, a 0–100 ms lag is fine; the GWS does not need the 900 ms.
- The 0x39A B4 P-lock value 2 (pawl) and 0x3F9 bits 28-29 must be **2 whenever you report P**, or the CAS terminal logic misbehaves.
- KL50/0x1B3: Sam's run 8 showed the 0x1B3 KL50 bit never set during ST_KL D (CAS sent 02 throughout), which matches the run-8 note. It has **no role in gear reporting**. [CONFIRMED per Sam/run-8 note]

## Open gaps / next captures
1. **N as a held selection:** the 0x39A B3 and 0x1AF B7 N values, and the FR 0x28/0x142 N values. Capture D→Up1 (N) and hold it for 5 s.
2. **Shift attempt without brake** (P→D, N→R) to confirm the shift-lock reaction, any 0x3FD blink, and the CC message.
3. **Lever moves in P without Unlock:** does the GWS really stay at 0F, and which "impossible action" frame does it send (openinverter mentions an extra message)?
4. **P button in P, double P press, and P at speed** (park-lock abuse threshold).
5. **Side/M gate and M+/M−** on F11: 0x197 B2 5/6/7, 0x3FD 0x81 and B3/B4.
6. **Meaning of the `SWI_Shiftlock_Disable` = 0x12 bits** (firmware xrefs into `ShiftLock.*`).
7. **D → R and R → P via lever / direct D ↔ R** (map predicts Up2 from D → R).
