# F11 emulator runs 10a / 10b (v1h.2): FR + PT-CAN check vs run 9 / run 8

Fred, Sun 1 Nov 2026 (Europe/Dublin, GMT). Inputs: `/workspace/f11-emu-run10/input/` (console logs) and Sam's replays in `/workspace/f11-emu-run10/replay/`, both read only. Capture times are in seconds from the first FR frame. "D" means ST_KL D: 10a 5.701 s, 10b 7.291 s, run 9 6.151 s, run 8 6.361 s.

The 10b console counters are cumulative with 10a (the device was not reset), so the 10b figures are 10b minus 10a. Byte numbering: FR bN is 1-based and CAN dN is 0-based.

Labels: **[C]** confirmed in data, **[L]** likely, **[S]** speculative. Numbers are in `f11_emu_run10_fr.csv`; the scripts are in `emu_run10_fr_scripts/` (`run_all.sh`).

## Bottom line
- v1h.2 fixes everything that was wrong in run 9:
  - Emulator TX misses: 0.03 % missing, against 18.9 % in run 9.
  - E6 gaps over 2 s: none now.
  - DTC events: 0, against 14 in run 9.
  - Stale gear frames, CAS P flips, 0x0E9 toggles and reverse-lamp flicker: all 0.
- One new defect remains, from the E6 change:
  1. **E6 event frames go out in odd cycles.** The real DDE never does this (0 of 1638 frames).
  2. **The 0x92 stop transition is no longer sent as an event.**
- The first defect delays ICM/DSC readiness by about 0.7 s on both starts. No ECU raised a fault.

## Health [C]
| Item | 10a | 10b | Run 9 |
|---|---|---|---|
| Header ISR max | 92 µs | 94 µs | 189 µs |
| lock_losses | 1 | 1 | 1 |
| USB fifo drops | 0 | 0 | 0 (run 8: 90 960) |
| Sequence gaps | 0 | 0 | |
| Logger crc_fail / ts_skipped | 639 / 17 664 | 536 / 13 636 | 1376 / 36 391 |
| PT-CAN TX fail / fifo-full | 0 | 0 | |

- Sam's check reports **FAIL: version mismatch** on both runs.
- The ISR >10 µs counter (10b) and the 70 µs guard counter (both runs) read 65535. These look like saturated 16-bit counters [S].

## 1. TX completeness
| Slot | 10a | 10b | Run 9 | Run 8 |
|---|---|---|---|---|
| 0x26 | 100 % | 100 % | 89.5 % | 99.7 % |
| 0x28 | 99.868 % (51) | 99.895 % (31) | 88.3 % | 98.0 % |
| 0x29 | 99.992 % (3) | 99.983 % (5) | 94.5 % | 98.8 % |
| 0x3D | 99.997 % (1) | 99.993 % (2) | 52.0 % | 83.0 % |
| 0x142 missing | 2 | 0 | 2 | 103 |

- The missing static frames equal the console miss + late counters exactly: 10a 0/51/3/1 + 0, 10b 0/30/5/2 + 1 late [C].
- Every dynamic variant is logged exactly as often as the console says it was sent [C]:
  - 0x5C, 0x68, 0xE6
  - E8 L4/L8
  - FB L4/L6/L8
  - 0x10A, 0x142
- Vehicle static slots are 1.68 % / 1.72 % missing, the same as run 9 (1.70 %). This is the logger's even-cycle residual, not the emulator [C value / L cause].
- USB drops are 0 [C].

## 2. Fault events [C]
- **No DTC events at all** in 10a or 10b (run 9: ICM D0156D ×11, EPS D51744 ×3).
- IDs 0x90/93/9A/A2/AB/B4/B7/CF carry only the normal ident burst. Counts per ID are 3/2/1/2/3/4/3/4, the same as the real captures.
- 0xBE has 0 frames.
- No new rare vehicle IDs appeared.
- PT-CAN 0x6xx: the same 4 IDs as runs 8 and 9 (0x601, 0x62A, 0x65E, 0x6F0) during the burst at D+10.4 s, plus 0x62A/0x6F0 at KL off. There is no fault traffic.

## 3. E6 timing
- **Gaps:** maximum gap 1.040 s in both runs, with no gaps over 2 s (run 9: 3.75 s and 4 gaps; run 8: 3.10 s and 2) [C].
- **Grid:** after D+0.5 s, 173 of 200 (10a) and 129 of 148 (10b) intervals are 1.000 ± 0.010 s. The rest are event frames [C].
- **Cycle parity:**
  - The real DDE sends E6 only in even cycles: 0 odd of 1638 frames in 7 captures [C].
  - The emulator sent 12 (10a) and 2 (10b) E6 frames in odd cycles. These include 12 of 16 content-change (event) frames in 10a. Each was copied into an even cycle only by the next grid frame, 45–959 ms later [C].
- **Stop:**
  - In both runs the stop frame (82/66) went in an odd cycle. **0x92 then waited for the grid: +715 / +735 ms** (Sam: +728 / +746 ms after ST_KL 8, FAIL; real +19 ms). Run 9 sent it at +5 ms.
  - The 92 phase now lasts 470–490 ms (run 9: 1200 ms; real ts_run4: about 1400 ms) [C].
  - No vehicle reaction differs in the bytes checked: 0x3A goes 22→72 23 ms after E6 80 (real +33), and DSC dropped before the stop as in run 9 [C]. Effects on other ECUs are unknown [S].

## 4. DSC / ICM / EPS at start
- **Start profile:** D lasts 200 ms in both runs (run 9: 220 ms; real: 860–2310 ms). 82 comes at D+176 / D+186, and 82→C takes 24 / 14 ms (real 8–39 ms) [C]. Unchanged short profile, and the car accepts it.
- **82 frame parity:**
  - The first 82 frame was in an **odd cycle (c23) in both runs**. The next 82 frame, in an even cycle, came at D+911 / D+881 [C].
  - ICM 0x3A b9 82→22 came at 82+768 / +728 ms. That is exactly +32 ms after the even copy. Real is 82+22…33, run 9 +33, run 8 +23 [C].
  - **DSC ready** (0x2F b4/b5/b7, 0x3F) came at 82+1768 / +1728 ms (real 1063–1082 in 4 starts and 2043 in 1; run 9 1072; run 8 1062). That is about 0.7 s late [C].
  - In all 4 emulator starts, 0x3A followed the first *even-cycle* 82 frame by 22–33 ms. So odd-cycle E6 frames are very probably ignored by the receivers [L].
- **Other start reactions [C]:**
  - 0x37 b6 at D+59 / +69 and 0x6C at D+135, earlier than real as before (follows rpm/E6 81) [L].
  - 0x113 at D+971 / +1061 and 0x44 at D+5039 / +5049 match.
- **0x0E9 c2 b5:** f1→f0 at D+91 / +121, then no toggles until it returns to f1 at the stop (run 9: 354 changes). 0x3FB base-content frames while running: 0 (run 9: 381) [C].
- **Ident burst:** at D+10.41–10.66 s / D+10.43–10.68 s, which is normal [C].
- **0xBE:** there is still no DDE answer to the burst or to the 0x9A "22 F101" request (D+20.9 s / D+15.5 s). The real DDE answers [C].

## 5. Gear [C unless noted]
- **0x3FD sequence:**
  - 10a: P, D 28.93, N 32.56, D 39.82, N 44.74, R 49.36, N 59.65, D 61.96, P 94.23. N was held 7.3 s.
  - 10b: P, N 38.05, R 38.11, N 48.58, D 53.89, P 68.22.
  - No flapping (run 9 flapped P/D).
- **Stale and flip checks:** 0 stale 0x39A P+pawl frames in R/D (run 9: 138). 0 CAS 0x12F d5 01→05 flips in R/N/D (run 9: 64). 0 FR 0x74 flips (run 9: 60). The E6 b4 0x16 blip: 0 (run 9: 2).
- **Early-0x3FD window:**
  - P→D: 0x39A follows +10 ms and E6 b7 +15 ms. N→D: 0 / +5 ms. N→R (10b): +12 / +18 ms. Run 9 was +65 ms.
  - The E6 *even-cycle* copy came up to 959 ms later in 10a (see section 3).
- **P button:** 0x39A and E6 show N for 0.88 s while 0x3FD still says D, then P. Run 9 showed the same pattern. Not compared with a real P-button capture [S].
- **0x142 b12:** 00→80 at D+1.4 s and then constant in all runs. It is not emulated (real: 82/84 in R, 88/98/90 in D).
- **Console "lever 0x197 lost (90 ms)":** 3 lines in 10a and 17 in 10b (12 while in R). On the bus, the longest 0x197 gap is 60.0 ms (10a) / 30.4 ms (10b), with 0 bad frames, and the gear was held. So this is an emulator-side false alarm [L].
- **Console "65524 older gear events":** shown at console 58.2 s, which is about 41.6 s in the capture. 0x3FD has no change between 39.82 and 44.74 s, so it is a display artefact only [C data / S cause].

## 6. Reverse lamp [C]
0x21A d1 bit0 has one on and one off per R interval and **0 flicker** (run 9: 24 offs in R). It switches on +307 / +316 ms after 0x3FD R and off +16 ms after leaving R.

## Immobiliser
The ZV never asks the car for an immobiliser exchange, and in testing the car never asked for one, so nothing is needed.

Our 0x592 (MIL off) went out every 5.1 s: 23 frames in 10a, 15 in 10b (run 9: 24).

## Must change
1. **Send E6 only in even cycles.** Put event frames in the next even cycle (≤ 5 ms delay) and keep the grid in even cycles.
   - Evidence:
     - The real DDE sends E6 only in even cycles: 0 of 1638 frames [C].
     - ICM and DSC readiness waited for the even copy, about 0.7 s late, on both starts [C timing / L cause].
     - 12 of 16 event frames in 10a (gear changes) reached even cycles only 45–959 ms later [C].
2. **Send the 0x92 stop transition as an event frame** (in an even cycle), as v1h.1 did. It now waits for the grid (+715 / +735 ms; Sam FAIL), and the 92 phase is cut to about 0.48 s.
   - No ECU reaction was seen in the checked bytes [C]. It is a regression from run 9 and the fix is cheap.

## Nice to have
- Fix the version string (Sam FAIL), and widen or clear the 16-bit ISR and guard counters and the gear-event display counter.
- Stop the false "lever 0x197 lost" alarms (the bus shows no gap >60 ms).
- Answer 0xBE (DDE ident) to the burst and to 0x9A 22 F101.
- Emulate 0x142 b12 gear values.
- Lengthen ST_KL D/crank toward real (≥ 860 ms); 0x37/0x6C currently react about 90 ms earlier than real.
- Check the 0.88 s N interlude on the P button against a real P-button capture.
- The 1.7 % vehicle-slot residual is on the logger side (even-cycle decoder), not the emulator.
