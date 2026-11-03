# F11 emu run 8 (v1g.2 fake start, no DDE, no EGS): start timeline, end of crank, chassis reactions

Author: Fred (executor), 2026-11-01. Labels: **[C]** confirmed in the captures, **[L]** likely (strong inference), **[S]** speculative.
Byte numbering: FR bN = payload[N-1] (1-based, SavvyCAN data[N]); CAN dN is 0-based. "D" = first vehicle FR 0x74 frame with ST_KL (b3 low nibble) = D.
"82" = first DDE/emulator 0xE6 frame with b3 = 0x82. Times are ms relative to the button pulse (0x74 b6 bit 7 rising), to D, or to 82.
Wall clock (Europe/Dublin, GMT): logger started 10:45:09 GMT; the crank was ≈10:45:25 GMT (logger t ≈ 16 s, approximate).

- Inputs: Sam's replay `/workspace/f11-emu-run8/replay/f11_emu8_v1g.csv` (+ `.can.csv`), read-only. Totals match the console `done:` line exactly
  (738,449 FR1 frames, 87,528 PT-CAN Rx, 40,452 own TX) [C]. Firmware dde-emu-v1g.2, `--emulate --can --can-tx --fake-start cas`.
- Real-DDE starts: engstartthenstop, engstart_2krevs, mitm_term2, ts_run4, obd1, 4log_3krev (PT-CAN only in 4log_3krev; obd1 bus 2 is sparse).
- Outputs: `f11_emu_run8_start_fr.csv` (617 rows: sections 1_timeline, 2_crank_end, 3_chassis_byte, 3_event_ids) and `emu_run8_start_scripts/` (`run_all.sh`).
- Logger caveat [C]: 0x74 is sent only every 20 ms (cycle ≡ 0 mod 4), and the logger dropped the 7.341 s frame, so the FR D→C edge in run 8 is only bounded (7.321–7.361 s). The PT-CAN copy 0x12F (20 ms) is used for the run 8 end-of-crank timing.

## 1. Start timeline

### 1.1 Run 8 (s from the first FR record; ms relative to D = 6.361 s)

| event | run 8 | real range (6 starts) | match |
|---|---|---|---|
| brake on (0x74 b7&3) | D−1190 | D−260 … −1941 (2krevs: brake held long before) | [C] |
| button pulse 0x74 b6 = 0x85 | D−390 | D−260 … −400 (term2: not seen) | [C] |
| ST_KL 9 / A / C (0x74) | D−190 / −160 / −100 | 9: −220…−241, A: −130…−160, C: −90…−111 | [C] |
| ST_KL D on 0x12F | D−9 | 4log: D+9 | [C] |
| CounterHi 1→3 (0x28 c3, 0x29, 0x3D b2 hi; 0x28 c1 b10 hi) | +53 / +63 | +53…+91 | [C] |
| CounterHi back to 1 | +163…+174 | +153…+173 (term2, ts_run4, obd1, 4log); held until 82 in the two Day-1 captures | [C] |
| E6 b2 hi (CounterHi) | 3 from +101 until 82 | 3 until 82 (Day-1) or back to 1 at +141…+161 | both variants exist [C] |
| E6 81/65, b7 0x13 | +101 | +110…+140 | [C] |
| first rpm (0x28 c1 b6..7 ×0.25) | +103 | +103…+133 | [C] |
| 0x28 c3 b5/b6 24 20 | +113 | +112…+143 | [C] |
| 0x29 b5 low nibble 2 | +113 | +112…+134 | [C] |
| 0x142 b23 = 24 | +156 | +151…+241 | [C] |
| 0x28 c3 b5/b6 20 20 | +213 | +194…+384 | [C] |
| 0x29 / 0x3D F9 7C | +213 / +214 | +192…+254 | [C] |
| crank rpm median / max | 167 / 468 | 135…223 / 336…470 | [C] |
| torque raw (0x28 c1) while cranking | 2560 | 2498…2553 | [C] |
| rpm ≥ 300 / 500 / 1000 | +743 / +803 / +963 | 300: +563…+2004; 1000: +834…+2284 | [C] |
| **E6 82** (and PT-CAN 0x3F9 d2 = 82 at +937) | **+941** | **+851 … +2302** | inside range [C] |
| E6 b4 at 82 | **66** for 255 ms, then 26 | **26 from the first 82 frame** in all 6 | **mismatch [C]** (no visible effect) |
| E6 b7 hi nibble 1→0 (start-ready) | 82+25 | 82+11…+20 | [C] |
| 0x142 b8 = 84 | 82+15 | 82−4…+81 | [C] |
| **ST_KL D→C** | 0x12F 82+30 (FR 82+20…+59, frame drop) | FR 82+8…+39; 4log 0x12F 82+30 | [C] |
| ST_KL C→A | 0x12F +180 after C | FR +100…+260 | [C] |
| FB L9 b2 ≠ 0 | D+2136 | D+2126…+2176 (4log +4115) | [C] |
| E8 L9 b1/b2 ≠ 0 | D+2936 | D+2186…+5295 | [C] |
| E8 rate pre / D..D+2 s / D+3..8 s (frames/s) | 1.0 / **1.0** / 15.2 | 0.7–2.0 / **4.0–7.0** / 8.0–15.4 | **mismatch during crank [C]** |
| FB L9 b3..4 = 00 00 | D+5136 | D+5127…+5156 (4log +8115) | [C] |
| FB L7 voltage min | 10.60 V (emulator-generated) | 8.7–11.5 V | [C] |

Real D→C on PT-CAN is only available in 4log_3krev; there it is 82+30 ms, identical to run 8 [C].

### 1.2 D duration and start-to-82 latency (ST_KL D on FR 0x74)

| capture | D duration | D→82 | first rpm→82 | 82→C (FR) | rpm at 82 |
|---|---|---|---|---|---|
| **run 8** | **≈980 (0x12F 6.352→7.332)** | **941** | **838** | **20–59 (0x12F: 30)** | ≈950 (emulator threshold) |
| obd1 | 860 | 851 | 737 | 9 | ≈1000 |
| mitm_term2 | 941 | 931 | 827 | 10 | ≈1000 |
| ts_run4 | 1310 | 1271 | 1148 | 39 | ≈900–950 (C came before rpm 1000) |
| engstart_2krevs | 1780 | 1761 | 1628 | 19 | ≈1000 |
| 4log_3krev | 2039 | 2020 | 1917 | 19 (0x12F 30) | ≈1000 |
| engstartthenstop | 2310 | 2302 | 2178 | 8 | ≈1000 |

## 2. What ended the crank

- **[C]** ST_KL left D (D→C) at 7.332 s on 0x12F (FR 0x74: between 7.321 and 7.361 s). That is **30 ms after the emulator reported running**: E6 b3 = 82 on FR at 7.302 s and 0x3F9 d2 = 82 on PT-CAN at 7.298 s (D+937). The rpm (0x28 c1, 0x0A5) crossed ~950 rpm in the same 20 ms.
- **[C]** In every real start, D ends 8–39 ms after the DDE's 82, while the D duration varies 4× (860–2310 ms). A fixed CAS timer cannot explain this. The user's "≈1 s crank" equals the measured 980 ms D duration.
- **[L]** The CAS ends the starter request when it sees the DDE "running" status (E6 = 82 on FR and/or 0x3F9 = 82 on PT-CAN). Supporting evidence: in ts_run4, D→C came 39 ms after 82 but **before** rpm reached 1000 (rpm ≥ 800 was already 227 ms earlier). So a plain rpm threshold fits worse than the status.
- **[S]** Which bus the CAS reads (FR 0xE6 or PT-CAN 0x3F9), and whether it also checks rpm, is open. In every capture, including run 8, 82 and ~950–1000 rpm arrive within ~20 ms of each other.
  - Decoupling test: send 82 while rpm stays at crank level (or the reverse), or send 82 on only one bus.
- **[C]** The emulator's 82 is **scripted, not tied to the real crank**. In `src/dde_engine.h` the fire is at `ENG_T_FIRE` = 700 ms, followed by a run-up of 200→1000 rpm over 250 ms, with 82 at ≥ 950 rpm. That gives 82 ≈ 934 ms after the trigger, matching the measured D+941. The real faulty engine never fired; the emulator's rpm alone ended the crank.

### Could reporting "running" sooner shorten the crank?

- **[L] Yes.** The crank lasts exactly as long as the emulator takes to report 82, plus ~30 ms of CAS reaction. Lowering `ENG_T_FIRE` shortens it one-for-one.
- **The minimum safe latency seen with the real DDE:** 82 at D+851 ms (obd1), i.e. 737 ms after the first rpm. The next fastest is D+931 (mitm_term2). No real start reported 82 sooner, so nothing earlier has been validated.
- **Recommendation [L]:** `ENG_T_FIRE` 700 → 600 ms (82 ≈ D+840 ms, at the fastest real start). This saves ~100 ms of cranking and stays inside real behaviour.
  - Going lower is **[S]** and untested. The CAS may enforce a minimum crank time, or check plausibility against the starter. Test in steps (e.g. 500, then 400 ms) and check that D→C still follows 82 within ~10–40 ms and that no 0x5C0/CC or DTC traffic appears.
  - Keep the run-up of ~250 ms. The real 300→1000 rpm run-up is 240–340 ms (ts_run4: 780 ms).
- **Secondary fixes [C mismatch, effect unknown]:**
  - At 82, send E6 b4 = 26 straight away rather than 66 for 250 ms. In the real starts, the 66/26 alternation appears only from 82+100 ms, in the Day-1 captures.
  - Raise the E8 L9 rate to ~5–7/s during the crank. It currently stays at 1/s until D+3 s.

## 3. Chassis ECU reactions (vehicle-side FR)

- **Method:** every vehicle frame (ID, length, cycle%4) byte with ≤ 8 values was compared over three windows: pre (D−2.5…−0.3 s), crank (D+0.3…+0.8 s) and post (D+4…+9 s). Bytes that changed in ≥ 3 of the 6 real starts were then timed per capture (`chassis.py`, `chassis_keys.py`).
- **Limitation:** there is no EPS or ICM FR frame map in our notes. EPS (node 0x30) and ICM (0x1C) are attributable only through their event/diagnostic frames. The scan therefore covers all vehicle frames by ID, without a sender label.

| byte | run 8 | real (fresh KL15 + start) | real (KL15 on long before) | verdict |
|---|---|---|---|---|
| DSC 0x2F b4 0e→02, b5 30→20 ("DSC ready") | 82+1062 | 82+1063 / +1074 / +1072 (4log +2043) | already 02/20; dips to 0e/30 at D−66…−77 | **match [C]** |
| DSC 0x2F b7 fe→f2 | 82+1062 | 82+1063…+1074 (4log +2043) | 82+624…+632 | **match [C]**; meaning [S] "engine running" |
| 0x3F b3 00→04, b4 fb→f3 | 82+1063 | 82+1072…+1082 (4log +2043) | already set | **match [C]** |
| 0x2F b8 brake bits | d0/d3/d7 follow the brake | same | same | match [C] |
| 0x37 b6 b0→a0 (crank) | D+159 | D+158…+238 | D+159…+169 | match [C] |
| 0x6C b4/b5 60/e0→30/24 (crank) | D+175 | D+214…+774 | D+175…+245 | match [C] |
| 0x3A c1 b9 82→22 | 82+23 | 82+22…+33 | 82+23…+33 | **match [C]**; reacts to 82 like the CAS [L] |
| 0x113 b4 fc→e0 | D+991 (82+50) | D+1011…+1101 (4log 82+31) | already e0 | match [C] |
| 0x44 b8 fe→f2, b12 e7→27 | 82+4078 | 82+3268…+5998 | 82+98…+128 | match [C] |
| 0x6D b5 f4→f2 | D+1780 (cycle 0) | D+2120…+3410 | D+170…+240 | similar [C] |
| 0x2C c3 b4/b6 7c/7a→7d (qualifier-like) | dip at the button/brake, 7d after | same | same | match [C] |
| 0x2B c3 (incl. b11/b12 e0/f8) | no start-related change | none | none | match [C] |
| **0x0E9 L6 c2 b5 f1→f0** | **stays f1 for the whole run** | D+170…+401 | D+341…+371 | **mismatch [C]**; sender and driving signal [S] |
| 0x131 L6 b1 00→02 | D+8601 (3 frames) | engstartthenstop only | – | rare frame; not judged |

- **Fault-report IDs [C]:**
  - There are **no DTC event records** (`62 17 04 <code>`) on 0x90, 0x93, 0xAB, 0xB4, 0xB7 or 0xCF in run 8, before, during or after the start. The same is true of every real start.
  - The only traffic on these IDs is the post-start identification burst at **82+10.2 s**. In it, the EPS (0x30, on 0xB7/0x93), ICM (0x1C, on 0xCF/0xAB) and DSC (0x29, on 0xB4/0x90) answer a 22 F101 read from tester F0.
  - Run 8 shows it at D+11.135…11.385 s (82+10.19…10.44 s). Real: 82+10.14…10.49 s, or 82+16.2 s in 4log.
- **0xBE [C]:** the real DDE also answers that burst on 0xBE (mitm_term2 +11190, ts_run4 +11435, 4log +18200 ms). **The emulator sends no 0xBE answer.** Whether anything then logs a fault is [S]; no DTC event followed within the capture.
- **Overall [C]:** the DSC, EPS, ICM and other vehicle-side reactions in run 8 follow the real-DDE pattern and timing relative to 82. The exception is 0x0E9 b5. **No chassis fault event was raised.**

## 4. Immobiliser

The ZV never asks the car for an immobiliser exchange, and in testing the car never asked for one, so nothing is needed. The starter cranked (ST_KL D for 980 ms) without one [C].

## 5. Recommended emulator changes (summary)

1. `ENG_T_FIRE` 700 → 600 ms: 82 at ≈ D+840 ms, the fastest real start (obd1 D+851). The crank shortens by ~100 ms [L]. Going further is [S]; test in steps.
2. E6 b4 = 26 from the first 82 frame [C mismatch].
3. E8 L9 at ~5–7 frames/s from D [C mismatch].
4. Investigate 0x0E9 b5 (f1 stays) and the missing 0xBE answer at 82+10 s [C mismatch, cause S].
