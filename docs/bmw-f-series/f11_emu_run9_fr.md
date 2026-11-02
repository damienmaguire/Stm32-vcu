# F11 emulator run 9 (v1h.1, short start + gear emulation): FlexRay view

Written Sun 1 Nov 2026 (Europe/Dublin, GMT). Raw data: `/workspace/f11-emu-run9/input/` (console log + 7z). Sam's replay folder and the other agent's `f11_emu_run9_v1h1.md` were read only and not modified.
Scripts: `emu_run9_fr_scripts/` (`run_all.sh` lists the order). Table of all findings: `f11_emu_run9_fr.csv`.

**Conventions.** t = seconds from the first FR frame (logger console time ≈ t + 6 s). D = ST_KL D at t = 6.151 s; "82" = our E6 b3 = 82 (running) at 6.332 s.
FR bytes bN are 1-based (p[N-1]). PT-CAN bytes dN are 0-based (Ian's B numbering). Labels: **[C]** confirmed in the data, **[L]** likely, **[S]** speculative.
Real references: six real DDE starts (engstartthenstop, engstart_2krevs, mitm_term2, ts_run4, obd1, 4log_3krev) and 4log_r10md10m for gears. The eld time base of r10md10m drifts by 10–20 s across seconds, so for it I use only value sequences and lags inside ~100 ms; long lags come from Ian's report.

## 0. Capture health
- Logger: usb_fifo_dropped 0, seq_gaps 0, notify/ring overflow 0, crc_fail 1376, ts_skipped 36 391 [C]. Run 8 had usb_fifo_dropped 90 960, crc_fail 487, ts_skipped 13. The meaning of ts_skipped is unknown, and the jump is worth asking about [S].
- Car: P/R/N/D, PDC, radar and EPS fine, bonnet up (operator report). The bonnet CC re-pops are covered in `f11_emu_run9_v1h1.md`.

## 1. Short start
| event | t rel. button | rel. D |
|---|---|---|
| ST_KL 9 / A / C | +60 / +150 / +200 ms | −240 / −150 / −100 |
| ST_KL D (FR 0x74; CAS 0x12F at −2) | +300 | 0 |
| E6 81 (starting), b2hi 3 | +306 | **+6** |
| rpm > 0 | +313 | **+13** |
| CounterHi 3 window on 28c3 / 0x29 | | +3 … +123 |
| E6 82 (running), b4 26, b7 03 | +481 | **+181** |
| rpm ≥ 1000 | +493 | +193 |
| ST_KL C (0x74 / 0x12F) | +520 | **+220** |

- **D lasted 220 ms and the crank (D→82) 181 ms** [C]. Run 8: D 1000 ms, 82 at D+941. Real D is 860–2310 ms, with 82 at D+851…+2020. Run 9 is about 4× shorter than the fastest real start.
- 82→C is 39 ms on FR and 43 ms by the console's own measure; real is 8–39 ms [C].
- E6 81 and rpm > 0 now come at D+6/+13, against real +110…140 and +103…133. Every start latency is earlier than any real start [C].
- **New:** no 0x3D frame with CounterHi 3 reached the bus. All 0x3D c3 frames between 6.135 and 6.255 s are missing (emulator TX misses, §3) [C]. 28c3/0x29 did show it. Effect: none visible [C], unknown for stored faults [S].

**Chassis reactions (run 9 / run 8 / real)**
- DSC ready, 0x2F c1/c3 b4/b5/b7 and 0x3F b3/b4: 82+1062…1073 / 82+1052…1063 / 82+1063…1082. **Match** [C].
- 0x3A c1 b9 82→22: 82+33 / — / 82+22…33. Match [C].
- 0x37 b6 b0→a0: D+59/69 / D+149/159 / D+148…179. 0x6C b4/b5 → 30/24: D+85 / D+175 / D+175…245. Both are earlier than real because they follow our earlier 81/rpm [C timing, L cause].
- 0x113 c2 fc→f0/f8/e0 at D−199 (real −99…−168), and 0x44 b8/b12 at D+5029/5039 (real up to 5029). Both match [C].
- Ident burst (functional 22 F101; EPS, ICM and DSC answer) at D+10 860…11 105 (82+10.68 s). Real D+10 990…18 169 (run 8 D+11 135) [C]; trigger unclear [S].
- **0x0E9 c2 b5 f1→f0 at D+61** (real +341…401; run 8 stayed f1): **fixed** [C]. From 45 s on it toggles with our stale 0x3FB (§2).
- **E6 b4 = 26 from the first 82 frame: fixed** [C]. Real also shows a b7 13 frame for ~20 ms before 03. We go straight to 03 [C, no visible effect].
- **E8 rate pre / crank / run: 1 / 5 / 16 frames/s** (real 0.7–2 / 4–7 / 8–15.4): **fixed** [C].
- **0xBE: no frame at all**, neither for the burst nor for the 0x9A "22 F101" request at t = 26.256 s (D+20 105). The real DDE answers. **Not fixed** [C].
- Changes not seen in any real start: 0x03B c0 b3 43→44 (D+4889), 0x03F c1 b15 brief 02 (D−286), 0x0FD L4 c2 b1 21→31 (D+941), 0x087 c3 b6, 0x0E9 c0 b2/b6. Seen [C], meaning unknown [S].
- **Is ~170 ms "running" flagged?** Only by our console ("CAS released START 43 ms after our running report (real 8–39 ms)").
  - No ECU broadcast a DTC event tied to the start [C].
  - CAS accepted it: ST_KL went to C at D+220 [C].
  - Whether something was stored silently (DME-plausibility type) is unknown; it needs a fault-memory read [S].

**Run-8 mismatches:**
- Fixed: E6 b4 66, E8 crank rate, 0x0E9 b5.
- Changed: crank 941 → 181 ms. It is now shorter than real rather than within it.
- Still open: no 0xBE answer.

## 2. Gear emulation
**What we send** [C]. There is no gear content on 0x142 b12: it stays 80. `dde_engine.c` only overrides b8/b23.

| shift (t) | driver | our PT-CAN | FR E6 b4/b7 | FR 28c3 b11/b14 | vehicle reactions |
|---|---|---|---|---|---|
| P→R | Up1 52.843, Up2 52.873 | 3FD 60 + 39A E1 + 3F9 d3 16 at 52.917; 3FD 40, 39A 02/E0, 3F9 06 at 52.982 | 16/03 at 52.920, 06/02 at 52.986 | 25/F4 at 52.993 | 0x107 b3 00 at 53.006; 0x106 L4 a1 at 53.041 (+59, real +56); 0x12F d5 / 0x74 b6 01 at 53.079 (+97 after 3FD R, real ≈ +40); 0x106 L6 b5 starts 53.096 (+114, real +131); 0x0F1 c1 a0 53.126; 0x21A d1 bit0 on at 53.285 (+303) and 0x0F2 rate 0.2→5/s |
| R→P | P button 73.783 | 39A gear 01 at 73.82; 3FD 20 at 74.697 (+914) | 06/01 73.820, 06/03 74.700, 26 at 76.820 (pawl, +3.04 s) | 22/FC 73.832, 22/FE 74.712 | 0x107 b3 02 at 74.725; 0x12F d5 05 shortly after the pawl |
| P→D | Down1 79.062, Down2 79.093 | **3FD 80 at 79.137, but 39A 03/E1 and 3F9 16 until 79.202** | 16/03 79.140, 06/05 79.205 | 24/F4 79.212 | 0x107 b3 00 at 79.225 |
| D→P | P button 90.072 | 3FD 20 at 90.987 (+915) | 06/01 90.110, 03 at 90.990, 26 at 93.110 (+3.04 s, timeout with brake on) | 22/FC 90.112, 22/FE 90.992 | 0x107 b3 02 at 91.005; 0x12F d5 05 at 93.115 (+5; real +20) |

**Lags against real** (Ian: 3FD +40 ms after the lever; P +0.9 s; pawl +3.0 s)
- 3FD +44 ms after Up2/Down2, P +914/915 ms, pawl +3.04 s [C]. Match.
- The FR codes match real r10md10m [C]:
  - E6 26/03 → 06/02 (R) → 06/05 (D);
  - "P coming" 06/01 then 03;
  - 28c3 25/F4, 24/F4, 22/FC → 22/FE.

**Vehicle reactions**
- 0x107 L4 b3 (copied 1 ms later to PT-CAN 0x2ED d2, and to 0x2DC d2) follows E6 b7 by +15…25 ms. Values: 00 in R/D, 02/03 in P [C].
  - Its 03↔02 bit moves with DSC 0x3F c1 b3 and 0x2F b8 at 49.426/94.545, the same as in real [L: bit = DSC state, not gear].
- 0x260 d4 also follows the DSC state, not gear [L].
- No EPS/EPB reaction to any shift was found [C, within the bytes scanned].

**Inconsistencies (all [C] as observations; consequences [S])**
1. **P→D: 0x3FD says D for 65 ms while 0x39A/0x3F9/FR E6/0x28 still say P/leaving P.** In real P→R, 3FD showed N during that phase. In real R→D, 3FD, 39A and E6 moved within 18 ms. No real P→D was captured.
2. **FR E6 b4 = 16 for 66 ms during P→R and P→D.** It copies 3F9 d3 16. In real P→R, E6 b4 went 26→06 directly while 3F9 showed 16.
3. **Stale PT-CAN frames carry base content (P + pawl).** There are 138 stale 0x39A frames in R/D (83 in R, 55 in D) and 2 stale 0x3FD P frames in D (79.247, 86.346). FR stays correct, so FR and PT-CAN disagree.
   - The car reacts: CAS 0x12F d5 flipped 01→05 (P) 64 times in R/D, 59 of them within 10 ms of a stale 0x39A (median 2.7 ms) [C]. FR 0x74 b6 flipped 60 times, 45 within 10 ms.
   - 0x0E9 c0/c2 b5 toggles f1/f0 with our stale 0x3FB d4 F1 frames (50 % of changes within 10 ms; chance 3 %) [C timing, L copy].
   - See also the other agent's counts (stale 0x3F9/0x397/0x3B3/0x3FB…).
4. **Reverse-lamp-like 0x21A d1 bit0 (FR 0x0F2 L4) toggled 24× on/off during R.** Real stays steadily on and goes off 41 ms after D [C]. The cause is not established: 50 % of toggles fall within 200 ms of a CAS P flip vs 31 % by chance (weak) [S].
5. 0x142 b12 is not emulated (real 82→84 in R, 88/98/90→80 in D) [C].

**N:** there was no held N in either capture. Ours: a 65 ms 3FD 60 transit with 39A E1 / 3F9 16 and FR E6 16/03. Real: a 60 ms transit with the same CAN bytes but FR E6 stays 26/03. The held-N code is untested [C].

**Not gear-specific:** 0x6C/0x2E0 30/24↔60/e0 episodes (6 in 44–100 s; also in run 8 and real 3krev) [C, trigger S]. Brief 20 ms transients on FR 0x2C c3 b3–b6 in D [C, meaning S].

## 3. Data loss
- **USB/logger loss is gone** [C]: usb_fifo_dropped 0, seq_gaps 0. 0x142 (ours, every 4th cycle): 2 missing out of 9304 (50.952 s, 127.290 s); run 8 had 99.
- Dynamic vehicle IDs 0x61/0x64/0x6C/0x70/0x87/0xFB/0x100 have 0 gaps (run 8: 29–225).
- **Vehicle-owned static slots: 1.70 % missing** (14 981 of 881 337; run 8 13.37 %, run 7 16.06 %) [C].
  - 0x3F is 99.45 % complete (run 8 81.3 %); 0x44 99.77 % (67.4 %).
  - The residual is flat over time (~80 misses/s), 90.6 % in even cycles, and mostly one slot per cycle. That matches the README's known "~2 % never decoded, even-cycle biased" decoder issue [L].
  - Worst IDs: 0x3B 93.4 %, 0x42 95.4 %, 0x5A/0x2F 96.6–96.7 %. 0x07/0x21 sit at ~97 % in every run.
  - Only 13.7 % of 0x3C misses fall in cycles where our 0x3D is also missing (0x3D miss rate 48 %). 0x3C is mostly lost when 0x3D *is* sent [C]; the mechanism is unknown (adjacent slot) [S].
- **Emulator-owned static slots regressed** [C]:

  | slot | run 9 | run 8 |
  |---|---|---|
  | 0x26 | 89.5 % | 99.7 % |
  | 0x28 | 88.3 % | 98.0 % |
  | 0x29 | 94.5 % | 98.8 % |
  | 0x3D | **52.0 %** | 83.0 % |

  - Overall 18.9 % missing vs 5.1 % in run 8.
  - Missing = console echo-miss + late exactly (0x28 3899 + 473, 0x29 1675 + 372, 0x3D 13 691 + 4182, 0x26 0 + 3898), constant over 0–186 s. So these frames were not transmitted; it is not a logger effect [C].

## 4. DTC events and diagnostics
- **14 DTC events (run 8: 0)** [C]:
  - **ICM 0xCF "D0156D" ×11**, each acknowledged on 0xAB: 10.501, 11.501, 25.501, 50.500, 53.500, 54.500, 55.500, 101.499, 104.499, 106.499, 107.499 s. Earlier analysis maps D0156D to ICM WMOM_DRV_3 = our 0x3D c3. In the 1 s before each event 62–88 % of 0x3D c3 was missing (run median 58 %) [L: caused by our 0x3D TX misses].
  - **EPS 0xB7 "D51744" ×3** (DT_PT_2 = our 0xE6), acknowledged on 0x93, at 55.155, 78.155, 81.655 s. Our 1 s periodic E6 frames pause after event frames, giving gaps of 2.62 s (52.986→55.610), 1.91 s and 2.41 s (79.205→81.610). Real maximum is ≈2.0 s [L for 55.155/81.655]. 78.155 does not fit (last E6 76.820), and run 8 had 3.1 s gaps with no event [S].
- Per-ID counts run 9 (run 8):

  | ID | count |
  |---|---|
  | 0x90 | 3 (2) |
  | 0x93 | 5 (1) |
  | 0x9A | 1 (1) |
  | 0xA2 | 2 (0) |
  | 0xAB | 14 (2) |
  | 0xB4 | 4 (2) |
  | 0xB7 | 6 (3) |
  | 0xCF | 15 (3) |
  | 0xBE | 0 (0) |

  Only 0xA2 is new: an FC in the burst, also in every real capture. No ID disappeared [C].
- PT-CAN 0x6xx is the same as run 8 [C]: 0x6F0 functional 22 F101 at D+10 841 → 0x601 (12), 0x62A (11), 0x65E (11); one more 0x6F0→0x62A at 126.37 s. No fault-memory read.

## 5. Immobiliser
The ZV never asks the car for an immobiliser exchange, and in testing the car never asked for one, so nothing is needed.

Check-control frame counts, run 9 / run 8: our 0x592 24 / 16, CAS 0x5C0 73 / 20, DSC 0x5A9 39 / 35, 0x5B8 2 / 2. The extra 0x5C0 frames fit the bonnet CC clear/re-pop [L].

## 6. Open questions / suggested next steps
1. **Emulator TX misses** (0x3D 48 %, 0x26/0x28 ~11 %) are new in v1h.1 and likely trigger the 11 ICM D0156D events [L]. Check the TX scheduling/CPU load added with gear emulation [S].
2. **Stale PT-CAN snapshots** (base content = P + pawl) make CAS see P in R/D, and maybe cause the lamp flicker. The fix should be to repeat the last good frame [L].
3. Order 0x3FD with 0x39A/0x3F9 on P→D. Mirror real by keeping E6 b4 26 during the E1 phase. Implement 0x142 b12.
4. Crank 181 ms is shorter than any real start. Consider ~850 ms (fastest real) [S]. Read DDE/CAS/DSC fault memory to check for silent plausibility faults [S].
5. Explain the E6 periodic pause after events (D51744) and the missing 0xBE answer. Meaning of ts_skipped 36 391.
