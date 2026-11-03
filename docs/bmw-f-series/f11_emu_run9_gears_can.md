# F11 520d emulator run 9 (v1h.1): gear selection on PT-CAN

The car had its real DDE and EGS removed. CAS, BDC, GWS, DSC, EPB and the PDC/radar modules were still fitted. The emulator (`pico-flexray-emu` d3ebc30, `fr_ts_logger.py --emulate --can --can-tx --fake-start cas --start-profile short`) played the DDE+EGS role on PT-CAN, with the gear driven by Damien's map. The bonnet was **up**.

Sam reported that gear selection was perfect and that PDC, radar and PAS all worked. This note is a CAN-side check of that run against the real EGS (`egs_gear_report_tx.md`, `egs-logs/F11_4log_r10md10m.csv`).

**Labels:**
- [CONFIRMED] = seen directly in the capture.
- [ANALYSIS] = derived from the capture.
- [HYPOTHESIS] = untested.

**Time base:** all times are seconds from the first CAN record of the capture (same base as Sam's `load9.py`). The console's "logger t" is about **+6.2 s** ahead of this base. For example, the console's "GEAR P→R" at 59.0 s is 52.9 s here.

Inputs:
- `f11-emu-run9/input/` (the 7z holds only the `.csv.bin`; it is byte-identical to `extract/`).
- Sam's replay `f11-emu-run9/replay/f11_emu9_v1h1.csv.can.csv`: 115 104 Rx frames (111 IDs) + 59 956 own Tx frames (19 IDs), no loss. Not modified.

Outputs:
- This file.
- `f11_emu_run9_gears_can.csv`: 623 rows. Columns: t_s, phase, source, can_id, event, value, dt_ms from the trigger, real_ms, label, note.
- Scripts in `emu_run9_scripts/`:
  - `load9.py`
  - `e2e_check.py`
  - `gears.py`
  - `cas_f42.py`
  - `cc.py`
  - `cc18_corr.py`
  - `tx_pairs.py`
  - `real_ref.py`
  - `dump.py`
  - `mkcsv.py`

## 1. Summary

1. **[CONFIRMED] The gear logic works.**
   - The car followed every gear the emulator selected: P→R, R→P (P button), P→D and D→P (P button).
   - The ignored lever inputs were also handled correctly: R + Up1 stayed in R, and D + Down1 stayed in D.
   - No gear-related check-control (CC) appeared on PT-CAN.
   - E2E checks found no errors on any frame (§2).
2. **[CONFIRMED] There is a major new v1h.1 bug: unpatched template frames.**
   - From engine start to the end of the capture, the emulator sends the raw `build()` templates in between the correct frames:
     - **0x3F9:** 1153 stale `80 64 3d 3d 23 47` frames, about 6 per second. Each one says engine not running, P-lock 2 and gear 3, and is followed 1–3 ms later by the correct frame.
     - **0x39A:** 159 stale `03/E2` frames (P, pawl in) while in R/D or P-coming, each corrected about 20 ms later.
     - **0x3FD:** 2 single-frame P glitches while in D.
     - **0x0DC:** 8 single-frame P glitches while in D.
   - **The car reacts to these frames:**
     - CAS 0x12F field42 ("P secured") goes to 1 every time, 1.8–4.2 ms after a stale frame: 76 of 76 rises.
     - The BDC's CC 18 (bonnet) clears and re-sets: all 32 clears fall 10–12 ms after a stale 0x3F9.
   - Root cause (§7): see below. [ANALYSIS]
3. **N verdict:** **not established.**
   - N was never selected and held. The only N was the emulator's own 65 ms 0x3FD=0x60 transit during P→R. The car did not object to it, but that is not acceptance of N.
   - **The value 0x39A N = 01 remains [HYPOTHESIS].**
4. **D→P pawl timing:**
   - P display came at +914.8 ms after the P button (real: +900).
   - The pawl was confirmed at +3033.8 ms on 0x3F9 and +3034.1 ms on 0x39A E2 (real: 3.0 s timeout).
   - The brake was still held, so this was the timeout path. **The brake-release path was not exercised.**
5. Deviations from the real EGS timing:
   - In P→R, R arrives about 70 ms late because N is always inserted.
   - E1 arrives about 55 ms late.
   - 0x39A's position relative to 0x3FD differs from the real EGS (§4).
6. **[CONFIRMED]** 0x3FD went out on PT-CAN only (the README says there is no PT-CAN2 TX). Run 9 is a single-bus log. Nothing on PT-CAN showed a need for PT-CAN2 (§6).

## 2. E2E / alive counters [CONFIRMED]

| ID | dir | frames | CRC8 (poly 1D, data ID) bad | counter 0..14 steps | cycle |
|---|---|---|---|---|---|
| 0x3FD | Tx | 1868 | 0 (ID 0x39) | always +1 | median 100.0 ms, max 102 ms. Event frames sit between grid frames; the grid is not moved |
| 0x3F9 | Tx | 2547 | 0 (ID 0x37) | always +1 | 1 s grid + events + **stale pairs** (real: ~1.4 frames/s; run 9: 13.7/s) |
| 0x197 | Rx | 6204 | 0 (ID 0x3F) | one anomaly at 33.353 s (P, idle): one frame ~65 ms early. [ANALYSIS] Arrival/logging artefact, not gear-relevant | median 30 ms in this capture (real 24 ms) |
| 0x12F | Rx | 2187 | 0 (ID 0x13) | ok | 100 ms + many CAS event frames in R/D (335 off-grid in 40 s, caused by field42 toggling) |

## 3. Lever / P-button timeline [CONFIRMED]

The brake (0x173 d7&3 = 3, 0x12F d6 = 0x33) was on from 29.66 s to 96.15 s, so **every shift was made with the brake pressed.** No no-brake attempt was made.

| t (s) | 0x197 | meaning |
|---|---|---|
| 52.8587 | B1hi 2, B2 1F | Up1 (lasted 1 frame, ~30 ms) |
| 52.8886 | B1hi 1, B2 2F | Up2 |
| 53.3086 / 53.3687 | 1F / 0F | Up1 again (in R), then idle |
| 73.7985 – 73.9785 | B3 D5 (6 frames) | P button (R→P) |
| 79.0784 | B1hi 3, B2 3F | Down1 |
| 79.1086 | 4F | Down2 |
| 79.4984 / 79.5584 | 3F / 0F | Down1 again (in D), then idle |
| 90.0884 – 90.2984 | B3 D5 (7 frames) | P button (D→P) |

0x197 only ever showed B2 low nibble F and B3 C0/D5. There were no other GWS states.

## 4. Per-transition comparison (emulator vs real EGS)

Latencies are in ms from the first lever notch or the P-button frame. Real values come from `egs_gear_report_tx.md` / `real_ref.py`. The real P→D and R→P were not captured, so the closest real transition is used. All emulator values are [CONFIRMED]; the verdicts are [ANALYSIS].

| transition | signal | emulator | real | verdict |
|---|---|---|---|---|
| **P→R** (Up1 → Up2 +30) | 39A 03/E1 + 3F9 P-lock 1 | +74.5 | +20 | **late ~55 ms** (emulator acts only after the 40 ms debounce on the deepest notch) |
| | 3FD N (0x60) event | +74.5 (+44.6 after Up2), lasts **65 ms** | +40, only because Up1 was held 61 ms | always inserted; with a 30 ms Up1 the real EGS probably goes straight to R [HYPOTHESIS] |
| | 3FD R (0x40) event | **+139.5** (+109.6 after Up2) | +100 (+39 after Up2) | **~70 ms late** |
| | 39A 02/E0 ; 3F9 B6=2 ; 0DC 25/F4 | +0.2 / −0.2 / +4.4 vs 3FD R | 39A +21 after 3FD | 39A not delayed (minor) |
| R + Up1 | — | no change | no change (map) | OK |
| **R→P** (P button) | 39A 01/E0 ; 3F9 B6=1 ; 0DC 22/FC | +34.4 ; +34.2 ; +44.1 | 39A +21, 3F9 21 ms after 39A | 39A ~13 ms late; 3F9 sent with 39A, not 21 ms after it |
| | 3FD P (0x20) event | **+914.5** | +900 | OK |
| | 39A 03/E0 ; 3F9 B6=3 ; 0DC 22/FE | +0.2 ; −0.2 ; +9.5 vs 3FD P | 39A 03 **19 ms before** 3FD, 3F9 +2 after | order deviates (minor) |
| | pawl: 39A E2 ; 3F9 P-lock 2 | +3019.2 ; +3034.1 | +3000 (timeout) / brake release +340 | OK. The E2 frame on the bus at +3019 is a stale template whose payload equals the new pawl state, so the intended event frame was suppressed. A corrected stale E2 also appeared earlier, at 75.958 s (+2159 ms) |
| **P→D** (Down1 → Down2 +30) | 39A 03/E1 + 3F9 P-lock 1 | +74.6 (+44.1 after Down2) | +20 (P→R ref) | late ~55 ms |
| | 3FD D (0x80) event | +74.6 (+44.4 after Down2), no N | +40 after Down2 (R→D ref) | **OK** |
| | 39A 05/E0 ; 3F9 B6=5 ; 0DC 24/F4 | **+64.8** / +64.6 / +69.6 vs 3FD D | 39A +2 after 3FD (R→D); +21 (P→R) | 39A gear lags 3FD by ~45–60 ms more than real |
| D + Down1 | — | no change | no change (map) | OK |
| **D→P** (P button) | 39A 01/E0 ; 3F9 B6=1 ; 0DC 22/FC | **+44.1** ; +34.6 ; +34.1 | 39A +21, 3F9 +42 | 39A was delayed by a stale 39A at +4.1 ms (+ its correction at +24) and the 20 ms event gap. Order inverted vs real |
| | 3FD P event | **+914.8** | +900 | OK |
| | 39A 03/E0 ; 3F9 B6=3 ; 0DC 22/FE | **+14.0** ; −0.2 ; −0.7 vs 3FD P | 39A 03 19 ms before 3FD | 39A again delayed by the stale frame at 90.978 s and the event gap |
| | pawl: 3F9 P-lock 2 ; 39A E2 | **+3033.8 ; +3034.1** (2119 ms after P display) | +3000; 39A then 3F9 +2 | OK (timeout path) |

Additional checks:
- **[CONFIRMED]** Every one of the 5 genuine 0x3FD changes went out as an extra event frame (N 52.9332, R 52.9982, P 74.7130, D 79.1530, P 91.0032). The 100 ms grid was not disturbed, which matches the real EGS behaviour.
- **[CONFIRMED]** The two extra 0x3FD glitch frames are listed in §7.
- **[CONFIRMED]** 0x0DC and FR 0xE6/0x28 followed the gear with no FR glitches.

## 5. N verdict

- **[CONFIRMED]** N was never selected and held. Sam did not run the README's "back 1 = N" test or the no-brake tests.
- **[CONFIRMED]** The only N on the bus was the emulator's synthetic 65 ms 0x3FD=0x60 during P→R, with 0x39A 03/E1 (P-lock released). 0x39A never sent 01 for N; 01 appeared only during the P-button transits.
- **[CONFIRMED]** The 0x3FD 0x60 transit value itself matches the real capture. No CC and no refusal appeared during the transit. A 65 ms transit is not evidence that the car accepts N.
- **[HYPOTHESIS]** 0x39A gear code for N = 01 (and 0x3F9 B6 nibble for N) is still open. It needs an N-hold test with the car's response observed.

## 6. PT-CAN2

- **[CONFIRMED]** Run 9 logged one CAN bus (logger bus 2 = PT-CAN). 0x3FD went out only there. The real EGS sends 0x3FD on both PT-CAN and PT-CAN2 within ~1 ms.
- **[CONFIRMED]** Real 4-bus log: 0x197 and 0x12F appear on both buses. Only 20 IDs are PT-CAN2-only: 0x0A0, 0x0A6, 0x0A7, 0x0B0, 0x0F8, 0x137, 0x145, 0x1AF, 0x1BA, 0x2B1, 0x335, 0x33D, 0x517 and 7 extended IDs.
- **[ANALYSIS]** All the consumers that showed a reaction (GWS, CAS, BDC, cluster, DSC) were served from PT-CAN, and the gear display/selection worked. **Nothing seen requires 0x3FD on PT-CAN2.** The PT-CAN2-only traffic in the real car looks like DDE↔EGS private traffic, and both of those ECUs are removed.
- **[HYPOTHESIS]** A remaining PT-CAN2 node might still expect 0x3FD. Logging PT-CAN2 in the next run would settle it.

## 7. The stale-frame bug (new in v1h/v1h.1)

**[CONFIRMED] counts.** Pairs = the same ID again within 5 ms.

| ID | P running 6.2–52.8 s | R/D 52.9–93.2 s | P 93.2–152.8 s | after stop |
|---|---|---|---|---|
| 0x3F9 pairs | 291 | 226 | 446 | 251 (alternating `80 64 23` / `80 64 13`) |

Other Tx IDs show 0–7 pairs. The 0x397 pairs match Sam's checker FAIL on "0x397 engine-not-running spacing". The other IDs are invisible because a CYC_EVENT frame only fires when its payload differs. Details:
- **0x39A:** a stale frame is only visible when the gear is not "P + pawl". There were 159, all in R/D/P-coming.
- **0x3FD:** P (0x20) single frames at **79.2625 and 86.3623 s**, both while in D. Each was corrected by an event frame about 21 ms later.
- **0x0DC:** 22/FE (P) single frames at 79.263, 84.123, 84.703, 85.523, 85.743, 85.823, 86.003 and 86.363 s.

**[ANALYSIS] Root cause.**
1. `ptcan_emu.c` sets `core.eng = dde_engine_snapshot(&eng_snap) ? &eng_snap : NULL`. When it is NULL, `pte_next()` skips `eng_patch_can()` and the raw `build()` template goes out.
2. `engine_read*()` (dde_emu.c) is a seqlock reader. It makes 8 tries back to back and `continue`s while the sequence number is odd, which means it does not really wait.
3. v1h/v1h.1 lengthened core 1's write window: it now holds the v2 status struct and `gear_fill_status()` (a 64-byte gear status with its event ring) between the two `eng_pub_seq` increments. A read on core 0 that lands in that window fails, so the snapshot returns NULL.
4. CYC_EVENT then fires an event for the template and another event when the next snapshot succeeds. That produces the pair.
5. Run 8 (v1g.2) used the same reader and had no pairs (161 0x3F9 frames in 143 s).

## 8. Reactions from other ECUs, classified

| ECU / frame | observation | class |
|---|---|---|
| **GWS 0x197** | Normal: only C0/D5 and the 5 lever positions. CRC/counter fine except the one blip at 33.353 s (P, idle). The GWS answered a diagnostic read at 17.0 s (0x65E) | other / none. [CONFIRMED] |
| **CAS 0x12F** | ST_KL steady 0x8A and brake byte 0x33 throughout R/D. **d5 bit2 (field42 "P secured") toggles 01↔05: 76 rises in R/D, every one 1.8–4.2 ms after a stale 0x3F9/0x39A**, back to 0 after a median 31 ms (min 24 ms). CAS sent 335 extra event frames | **gear-related, emulator-caused** [CONFIRMED correlation]. [ANALYSIS] Risk: CAS intermittently believes P is secured while in R/D (it matters for KL15-off / Terminal logic) |
| **BDC CC 18 (bonnet) on 0x5C0** | Set at 6.45 s and refreshed (as in run 8), but with **32 clear → re-set pairs (re-set 100–200 ms later)** at irregular times through P, R and D. **All 32 clears come 10–12 ms after a stale 0x3F9** (random-time median would be 118 ms). Final clear at 122.45 s | **bonnet (expected); the chatter is emulator-caused** [CONFIRMED correlation, ANALYSIS cause: stale B2=0x80 "engine not running"] |
| BDC CC 14 (door) 0x5C0 | active 13.75–14.25 s and 15.05–15.55 s (door opened, before any shift) | other |
| CC 148 0x5C0 | inactive at 2.55 s (pre-start, as run 8) | other |
| 0x592 | only the emulator's own CC 34 (MIL) **inactive**, every 5.1 s (24 frames) | emulator |
| **DSC 0x5A9** | CC 36/53/57/184/452/654 all **inactive**, sent once at 0.5–0.9 s. No DSC CC during the run | other (boot) |
| **EPB** | no EPB CC on PT-CAN. Nothing gear-related seen | none [CONFIRMED absence] |
| 0x581 (node 0x01) | CC 77 active 6.15–46.25 s and 97.35–122.75 s (likely seat-belt reminder [HYPOTHESIS]). CC 181 5.95–11.25 s | other (not correlated with shifts) |
| 0x590 (node 0x10) | CC 485 active from 27.93 s every ~6.3 s, cleared 122.65 s (same as run 8) | other (pre-existing) |
| 0x5B8 (node 0x38) | CC 45/245 inactive at 2.3/2.8 s | other |
| 0x5F2 (node 0x72) | CC 120/256/378 active, refreshed every 5.1 s from boot | other |
| **PDC / radar / PAS** | No PT-CAN CC or reaction. Sam reports all working; their traffic is not on PT-CAN | none |
| **Gear-related CC** | **none** (no EGS CC 419, no gear/selector fault CC on PT-CAN) | — |
| Diag 0x6F0 → **0x18 (EGS)** | **26.789 s, service 0x22, unanswered** (no 0x618). Also → 0x17 at 27.309 s, unanswered (both as run 8). A 17.0 s read round was answered by nodes 0x01, 0x2A and 0x5E (GWS); a 126.4 s read was answered by 0x2A | other; [HYPOTHESIS] harmless, but a 0x18 diag stub would silence it |

## 9. Fixes for v1h.2

1. **(must) Stale frames.**
   - On a failed snapshot, keep the last good snapshot instead of returning NULL. For example, cache `eng_snap` and use it while it is less than ~50 ms old.
   - Never transmit raw templates while the engine/gear overlay has been active since the last wake.
   - Move `gear_fill_status()` and the status-struct fill out of the seqlock write window (they need their own seqlock or a double buffer), so the window covers only `eng_pub`.
   - Make the reader retry with a short bounded wait (e.g. `tight_loop_contents()`, ≤20 µs) rather than 8 back-to-back tries.
   - This removes the CAS field42 toggling and the CC 18 chatter, plus the 0x3FD/0x0DC P glitches in D.
2. **P→R:**
   - Show N only if Up1 has been stable for ≥40 ms (as the real car did).
   - Otherwise go straight to R about 40 ms after the deepest notch. That would make R come about 40 ms after Up2 instead of about 110 ms.
3. **E1 / P-lock 1:** send them about 20 ms after the first lever notch, before the gear decision.
4. **0x39A ordering:**
   - In P→R/P→D, put the gear code ~2–21 ms after 0x3FD rather than holding E1 for 65 ms.
   - On the P button, send 0x39A 01 ~21 ms after the press with 0x3F9 B6=1 ~21 ms later.
   - Send 0x39A 03/E0 ~19 ms **before** 0x3FD P.
   - Note that the 20 ms 0x39A event gap delays these frames once a stale frame has used it.
5. **0x39A event compare:** compare against the last *intended* payload, not the last transmitted one, so a stale frame cannot suppress a real change (as happened with the pawl E2 at 76.83 s). This is optional once fix 1 is in.
6. **PT-CAN2:** log it in run 10. Mirror 0x3FD there only if a PT-CAN2 consumer shows up.
7. **Untested paths for run 10:**
   - N hold (back 1 from R, and from D), with the car's response observed. This is the only way to confirm 0x39A N = 01.
   - Shifts without the brake (expect a refusal).
   - The pawl on brake release (expect +340 ms after release).
   - Optionally an R→D shift without passing through P.
8. **(optional)** Add a minimal 0x618 negative response to the 0x6F0→0x18 read, only if quiet diagnostics matter.
