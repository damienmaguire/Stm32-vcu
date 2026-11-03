# F11 emulator run 10 (dde-emu v1h.2): gear-selector tests on PT-CAN, runs 10a and 10b

Car: F11 with the DDE and EGS removed. The emulator (dde-emu **v1h.2**) is the only DDE/EGS on PT-CAN.
Captures:
- **10a**: `f11_emu10_v1h2`. 194 s, 119 038 Rx frames plus 54 823 own Tx frames.
- **10b**: `f11_emu10_v1h2_2`. 90 449 Rx frames plus 41 581 own Tx frames. Same firmware session as 10a.

Both are read from `f11-emu-run10/replay/*.csv.can.csv`, read-only. That data came from the fr_ts_logger v1h.2 replay decode, with no frames lost.
Timestamps: **t = seconds from the first CAN record of each capture.** Console "logger t" is about +16.7 s later in 10a and about +19.7 s later in 10b.

Labels used below:
- [C] = directly in a capture.
- [A] = analysis.
- [H] = hypothesis.

Real reference: `egs-logs/F11_4log_r10md10m.csv`, bus 3 (real DDE plus EGS). Run 9: `f11_emu_run9_gears_can.md`.
Scripts: `analysis/emu_run10_scripts/`. Timeline: `f11_emu_run10_gears_can.csv`, with one row per signal change and a `lever_ms` column.
Sam's checker (`dist/dde-emu/v1h2/dde-emu/check_emu_capture.py --engine --gear --flicker`) gives **2 FAILs in each run**. The output is in `emu_run10_scripts/check_emu_capture_10{a,b}.txt`. Both FAILs are covered in Input surprises below.

## Input surprises (read first)

1. **Logger/firmware mismatch [C].** The car ran logger **v1h.1** against firmware **v1h.2**, and both consoles print `VERSION MISMATCH`.
   - The new v1h.2 counters (snapshot-read failures, background-step max) are missing from the console.
   - In 10a the console shows `(65524 older gear events not shown)` at logger t = 58.2 s. It then reprints P→D, D→N, lost/back and N→D, all stamped 58.4 s.
   - CAN shows no shifts at 58.4 s. This is a console decode artefact [A].
   - The CAN/FlexRay replay decode itself is clean (v1h.2 decoder, 0 drops).
2. **10b is not a fresh boot [C].** The firmware counters carry over from 10a: changes 8→13, refusals 0→13, 0x197 ok 6460→11 394. The 0x197 ok delta (4934) equals exactly the 4934 frames of 0x197 on the bus in 10b.
3. **"lever 0x197 lost (no valid frame for 90 ms)" appears 3× in 10a and 17× in 10b** (10b mostly in R with the brake off, CAN t ≈ 39.9–47.6 s).
   - On the bus, 0x197 never has a gap. dt is 30.0 ms with a max of 30.4 ms in 10b. 10a has one 60 ms gap at 137 s, which is after engine stop. CRC is 0 bad in both runs.
   - The firmware ok-counter equals the bus count, so no frame went missing.
   - **These are false timeouts [A].** Likely causes are an internal stall in the gear/background task or a v1h.1 decode artefact [H].
   - No effect is visible in TX (gear held, no flicker), but this has to be re-checked with a matched logger.
4. **PT-CAN2 was not logged [C].** Both captures contain only logger bus 2 (PT-CAN). The v1h.2 README says the V2 board has one CAN controller, so a PT-CAN2 log needs a second adapter.
5. **Checker FAIL: engine-stop event on FlexRay 0xE6 [C].** 0xE6 b3 = 0x92 arrives **+728 ms (10a) / +746 ms (10b)** after ST_KL 8. The real DDE takes +19 ms.
   - On PT-CAN, 0x3F9 B2 82→92 is on time: 10a 112.907 is +7 ms after ST_KL 8 at 112.90.
   - So the FlexRay event copy of 0xE6 seems to wait for the 1 s grid [A]. This is not a gear issue, but it looks like a v1h.2 regression to check.
6. **CC 18 is active the whole time the engine runs, with the bonnet closed [C].** On 0x5C0 it is set at running start and cleared at stop:
   - 10a: 5.95–112.65
   - 10b: 7.57–76.27
   - No clear/re-set in between.

   The real-DDE start capture also shows CC 18 (`dde_start_sequence.md`, +1043 ms). So CC 18 follows "engine running" plus something CAS/BDC sees, perhaps the bonnet contact reading open [H]. The run-9 explanation ("bonnet was up") needs revisiting.
7. **Minor [C]: 0x397 in 10a only.** There are 31 identical duplicate frames, each about 3.5 ms after a grid frame, every 3.000 s (13.43 … 106.43 s). None appear in 10b, and the cause is unknown. The checker counts no 0x397 flicker. 0x0DC in 10a has one 25 ms gap every 15 s (17.60, 32.60, 47.60 s).

## Test log (what was actually done)

| run | t (s) | action | brake | result |
|---|---|---|---|---|
| 10a | 5.72 | start (ST_KL D) | – | running (0x12F ST_KL A 6.01–112.62) |
| 10a | 28.905 | P→D, Down1 (held 1650 ms) | on | D |
| 10a | 32.535 | D→N, Up1 | on | N, held 7.26 s; brake released 37.67–39.07 while in N |
| 10a | 39.795 | N→D, Down1 | on | D |
| 10a | 44.715 | D→N, Up1 | on | N, 4.62 s |
| 10a | 49.335 | N→R, Up1 (Up2 +60 ms; Up1 again 50.32) | on | R; the extra pushes are correctly ignored |
| 10a | 59.625 | R→N, Down1 | on | N, 2.31 s |
| 10a | 61.905 | N→D, Down1 + Down2 @+30 ms | on | D |
| 10a | 93.345 | D→P, P button | on (released 113.23) | P; pawl by **timeout** +3002 ms |
| 10a | 112.90 | stop (ST_KL 8) | | |
| 10b | 7.32 | start | – | running 7.62–76.18 |
| 10b | 21.06–36.24 | 8 lever episodes in P (Up1/Up2, Down1/Down2) | **off** | all refused (13 console refusals) |
| 10b | 38.045 | P→R, Up1 (60 ms) + Up2 | on | N transit 58.7 ms, then R |
| 10b | 48.575 | R→N, Down1 | **off** | accepted; N 5.31 s |
| 10b | 51.305 | N→D, Down1 + Down2 | **off** | refused |
| 10b | 53.855 | N→D, Down1 + Down2 @+30 ms | on | D |
| 10b | 67.355 | D→P, P button | on (released 75.68) | P; pawl by **timeout** +3002 ms |
| 10b | 76.46 | stop | | |

Brake on 0x173 d7&3 (= 0x12F d6 = FlexRay 0x74 d6, all within 3 ms) [C]:
- 10a: 23.38–37.67, 39.07–41.10, 43.58–56.44, 58.92–63.58, 73.34–89.09, 92.50–113.23.
- 10b: 6.02–9.78, 37.25–43.10, 52.78–55.84, 66.24–75.68.

## 1. Per-transition timing

All values are ms after the triggering 0x197 edge. "after Down2/Up2" means measured from the second edge. ⚠ = more than ~15 ms off the real EGS.

| transition | run | E1 (0x39A B4) | 0x3FD B2 | 0x39A B3 gear | 0x3F9 | 0x0DC B2/B5 | real r10md10m | run 9 |
|---|---|---|---|---|---|---|---|---|
| P→D Down1 | 10a | +23.0 | D +42.4 | 05 +52.4 (+10 after 3FD) | +22.8 (bits 1) / B6=5 +52.2 | 24/F4 +57.8 | (P→R) E1 +20, gear +40, 39A +21 after 3FD | E1 +74.5, 39A +64.8 after 3FD |
| P→R Up1 60 ms + Up2 | 10b | +22.1 | N +42.0, R **+40.7 after Up2** (N shown 58.7 ms) | 02 +112.4 (+11.7 after R) | +111.9 | 25/F4 +118.3 (stays 22/FE during N) | E1 +20, N +40, R +39 after Up2, N 60 ms, 39A +21, 0DC +22 | R +139.5 |
| D→N Up1 | 10a ×2 | – | +42.9 / +42.0 | 01 +43.1 / +42.1 | +42.7 / +41.7 | 22/FE +47.6 / +48.0 | (no real D→N) | – |
| N→D Down1 | 10a | – | +41.8 | +42.0 | +41.6 | +47.6 | ~+40 | – |
| N→R Up1 | 10a | – | +41.7 | 02 +41.9 | – | 25/F4 +47.7 | +40 | – |
| R→N Down1 | 10a / 10b | – | +42.6 / +42.2 | +41.7 / +42.4 | – / +42.0 | +57.8 / +48.2 | – | – |
| N→D Down1+Down2 | 10a / 10b | – | +71.8 / +72.2 (= **+41.8 / +42.2 after Down2**) | +72.0 / – | – | +77.8 / +88.1 | R→D: D +40 after Down2, 39A +2, 0DC +21 | – |
| D→P P button | 10a | – | **P +902.2** | 01 +22.8, 03 +882.6 (−19.6 before 3FD) | B6=1 +43.0, 3 +901.5 | 22/FC **+57.9 ⚠**, 22/FE **+918.2 ⚠** | 39A 01 +21, 03 +881, 3FD P +900 (39A 03 −19 before 3FD), 0DC FC +41, FE +1 after 3FD | 39A 01 +44.1, P +914.8, 39A 03 +14 after |
| D→P P button | 10b | – | P +902.7 | 01 +22.4, 03 +882.5 (−20.2) | +43.6 / +902.4 | 22/FC +48.2, 22/FE +908.6 (+6 after 3FD) | as above | |
| pawl (timeout) | 10a / 10b | E2 +3002.3 / +3001.7 | – | – | bits 2 +3001.5 / +3001.5 | – | real: E2 → CAS d5 05 +21 | – |
| CAS 0x12F d5 01→05 after E2 | 10a / 10b | +20.7 / +6.4 | | | | | +21 | |

**Verdict [C/A].** Every main signal (E1, 0x3FD, 0x39A gear, 0x3F9, P-button 900 ms sequence, Up2/Down2 second-edge rule) is within about 3 ms of the real EGS. This is a large improvement on run 9: E1 went from +74.5 to +22 ms, and the 0x39A lag after 0x3FD went from +65 to about +10 ms.
- The one flagged item is **0x0DC on the P button in 10a**: FC +58 vs real +41 (+17), and FE +16 after 0x3FD vs real +1. In 10b it is FC +48 (+7) and FE +6 after, which is within tolerance.
- 0x0DC is on a 20 ms grid with an occasional 25 ms slot, so this is probably grid phase plus about 5 ms processing [A]. Not a blocker.

## 2. Flicker counts

All [C], zero unless stated. Running windows: 10a 5.884–112.907, 10b 7.501–76.464.

| check | 10a | 10b |
|---|---|---|
| 0x3F9 run-9 flicker template / B2≠82 while running | 0 / 0 | 0 / 0 |
| 0x39A E2 while not in pawl (10a 28.927–96.346, 10b 38.066–70.355) | 0 | 0 |
| single-frame outliers on 0x3FD, 0x0DC (B2, B5), 0x39A, 0x3F9 (own script plus checker) | 0 | 0 |
| frame pairs <5 ms on gear IDs | 0 | only legitimate: 0x3F9 82→92 stop pair at 76.46; pre-run wake 0x0D9/0x0DC/0x0F3 at 3.098 |
| CAS 0x12F d5 bit2 flips in R/D/N | 0 | 0 |
| CAS 0x12F d5 edges | one 05→01 leaving P (28.963, 6 ms after 39A gear) and one 01→05 at pawl | 05→01 at 38.164 (after 39A 02), 01→05 at pawl |
| 0x12F off-grid (event) frames while out of P | 18 in 67 s | 6 in 32 s (run 9: 335 in ~40 s) |
| CC 18 clear→re-set | 0 | 0 |
| 0x397 single-frame flicker (checker) | 0 (31 exact duplicates, see surprise 7) | 0 |

## 3. N held

N periods:
- 10a: 32.578–39.837 (7.26 s, brake released 37.67–39.07 while in N), 44.757–49.377 (4.62 s), 59.667–61.977 (2.31 s).
- 10b: 48.617–53.927 (5.31 s, brake **off** until 52.78).

Emulator values in held N [C]:
- 0x3FD B2 = 60.
- 0x39A B3/B4 = 01/E0.
- 0x3F9 bits 28-29 = 0, B6 = 1.
- 0x0DC B2/B5 = **22/FE**.
- FlexRay 0xE6 b4/b7 = 0/1.
- FlexRay 0x28 = 22/FE.

The four sources agree in all 183 (10a) / 141 (10b) checker samples.

Car reaction [C]:
- CAS 0x12F d5 stays 01 (not P-secured); no flicker.
- GWS 0x197 keeps reporting normally; the next shift out of N is accepted every time.
- **0x2ED d2 and 0x2DC d2 go to 02 in N**, the same as in P. They are 00 in R/D and 03 after P is secured.
  - Latency after 0x3FD: +33–46 ms for 0x2ED and +77–106 ms for 0x2DC.
  - The real car shows the same 00 in R/D and 02 in P. A real N was never held long enough to compare.
  - Meaning [H]: a "not in drive gear / hold" status from the node behind FlexRay 0x107 (`f11_emu_run9_fr.md`). 02→03 after pawl came at +3.5 s in 10a and +0.12 s in 10b; in the real car it came 0.38 s *before* E2. Cause unknown.
- One CC, **0x5DE CC 717**, became active once while in N in 10a (46.18–47.39). See item 7.
- No DSC/EPB/KOMBI CC on any N hold, including the 10b N with the brake off.

**What this proves and doesn't prove [A].**
- It proves the car's own nodes tolerate N held for 2–7 s with these values: no shift lock-out, no warning CC from DSC or CAS, and the selector keeps working.
- It does **not** prove that 0x39A 01 or 0x0DC 22/FE are what a real EGS sends in a held N. The real capture only has a 60 ms N transit:
  - In that transit, 0x0DC stayed 22/FE and 0x39A went straight from 03 to 02, so there is no 01.
  - "P coming" uses 01 with 0x0DC 22/FC.
- So the code 0x39A 01 is shared by "N" and "P coming" in our emulation. Whether real N uses 0x0DC FC or FE, and 0x39A 01 or something else, is still open [H]. No consumer has been seen to distinguish them, and DTCs were not read.

## 4. R→D direct

**Not exercised in either run.** Both runs went R→N, then a separate N→D (Down1 then Down2 at +30 ms).
- What is characterised: the Down2 second-edge rule. D comes +41.8 ms (10a) / +42.2 ms (10b) after Down2; the real R→D gives +40.
- 0x39A follows in about 0 ms and 0x0DC in +6 ms (10a) / +16 ms (10b).

R→D direct (Down2 from R with the brake on) is still a test to run.

## 5. Pawl on brake release vs +340 ms

**Not exercised.** In both runs the brake was held for more than 3 s after the P button:
- 10a: released 19.9 s after
- 10b: released 8.3 s after

So the pawl came from the **timeout path**: E2 +3002.3 / +3001.7 ms, 0x3F9 bits 2 at +3001.5. CAS confirmed +20.7 ms later in 10a (real car +21) and +6.4 ms later in 10b. That is CAS phase on its 100 ms grid [A].

The brake-release path, and its target of about +340 ms after release, still needs a test: P button, then release the brake within about 1–2 s.

## 6. Refusals

- **Brake refusals by the emulator [C].** 10b has 8 lever episodes in P with the brake off (0x173 = 0, 0x12F d6 = 30), at 21.06, 23.13, 27.57, 33.45, 34.47, 35.01, 35.82 s, plus 51.30 s in N (N→D).
  - None caused any TX change.
  - That gives 13 console refusals, which matches the counter going 0→13.
  - No refusals in 10a: every push had the brake on.
- **R→N with the brake off was accepted** (10b 48.575). This matches OEM behaviour, where taking N needs no brake [H].
- **Unlock refusal by the GWS cannot be seen on CAN.** 0x197 has no unlock bit, and the GWS gates the lever mechanically before it reports a move (`egs_gear_report_tx.md`).
  - Every move that appeared on 0x197 was either accepted or refused by the emulator's brake logic.
  - A move blocked by the GWS shift lock is invisible by construction.
  - Telling the two apart needs an annotated test, with a note of when the lever physically would not move.
- **No "push brake" CC was shown** on either refusal type: no EGS CC and no emulator CC. This matches the build, since the EGS is removed and the emulator only sends CC 34 inactive. KOMBI had nothing to display [A]. If an OEM-like prompt is wanted, the emulator would have to send it, and the CC number is unknown.

## 7. Check-Controls by node and other reactions

CC frames have the form `40 lo hi st`, with odd st meaning active [C].

| ID (node) | CC | 10a | 10b | expected? |
|---|---|---|---|---|
| 0x581 (0x01) | 77 | active run-long | same | yes (seat belt [H]) |
| 0x581 | 181 | start-phase only | same | yes |
| 0x590 (0x10) | 485 | active from 27.26 | from 24.48 | yes (known 485) |
| 0x592 (emulator) | 34 | inactive, every 5.1 s (23 frames) | 15 frames | yes (MIL off) |
| 0x5A9 (DSC) | 36/53/57/184/452/654 | inactive, boot only | same | yes (pre-existing) |
| 0x5B8 (0x38) | 45/245 | inactive | inactive | yes |
| 0x5C0 (CAS/BDC) | 14 door | 6.45–10.75 | – | yes |
| 0x5C0 | 18 | 5.95–112.65 | 7.57–76.27 | expected but cause unclear (surprise 6) |
| 0x5C0 | 148 | inactive | – | – |
| 0x5F2 (0x72) | 120/256/378 | lights, pre-existing | same | yes |
| **0x5DE (GWS, 0x5E)** | **717** | **active 46.18–47.39** | **active 29.00–29.57** | **NEW** |

**CC 717 from the GWS [C/H].**
- 10a: in N with the brake on, about 1.1 s after the D→N Up1 was released (45.07).
- 10b: in P with the brake off, about 0.86 s after a refused Up2 was released (28.14).
- It was not repeated for identical later actions. It is not in the public CC lists, and its meaning is unknown.
- Hypothesis: a selector hint (e.g. "shift lock / press brake"), or a GWS reaction to something it expected back from the EGS [H].
- To pin it down: photograph the KOMBI during the next run, and read GWS fault memory.

**Not seen:** no EGS CC, no EPB CC, and no "push brake" CC.

**Diagnostics [C].**
- 0x6F0 → node 0x18, service 0x22, unanswered: 10a 25.589, 10b 23.330.
- 0x6F0 → 0x17, unanswered: 26.109 / 23.850. The emulator has no diag stub.
- The read round to 0x01, 0x2A and 0x5E was answered at about 16.1 s / 17.7 s.
- No retries storm. Nothing reacted visibly to the missing 0x18 answer [A].

**"Running" confirmed [C].**
- 0x12F d2 low nibble (ST_KL) = A from 6.01–112.62 (10a) and 7.62–76.18 (10b), going D→C→A at start and 9→8 at stop.
- 0x3F9 B2 = 82 throughout, and FlexRay 0xE6 b3 = 82.
- The console says CAS released START 24/23 ms after our running report.
- The checker confirms 0x82 is sent before CAS releases the starter.

## 8. CRC / alive counters

E2E: CRC8 poly 0x1D over data-ID lo, hi, B1…; the counter is the low nibble of B1, 0..14. All [C].

| ID | dir | 10a frames | 10b frames | CRC bad | counter | cycle |
|---|---|---|---|---|---|---|
| 0x3FD (DID 0x39) | Tx | 1946 | 1481 | 0 | all +1 | 100 ms (max 101; 10b one 164.7 ms at 3.098 startup); gear changes go out as extra off-grid frames, grid preserved (100.41 % slots) |
| 0x3F9 (DID 0x37) | Tx | 220 | 170 | 0 | ok | 1 s + events |
| 0x197 (DID 0x3F) | Rx | 6461 | 4934 | 0 | 10a: one blip at 137.229 (after stop; dt 6/25/60 ms); 10b clean | 30 ms |
| 0x12F (DID 0x13) | Rx | 1984 | 1515 | 0 | ok | 100 ms + events |

The checker also finds 154 949 static E2E frames with 0 bad CRC, and 0xE6 with 0 bad (10a).

## 9. Blockers before ZombieVerter vs nice-to-have

**Blockers (should be closed first):**
1. **Rerun with a matched logger (v1h.2) and firmware.** This is needed to read the new counters and to explain the **false "0x197 lost" timeouts** (20 in total, with 0 bus gaps). If they are real internal stalls, they would hold the gear during a stall and could mask lever input in a real drive.
2. **Pawl on brake release** (target about +340 ms after release) is untested, as is **R→D direct**.
3. **Held-N encoding.** It is unverified whether the real EGS sends 0x39A 01 and 0x0DC 22/FE or FC in a held N. Get a real-EGS capture with N held for at least 2 s, or read DSC/GWS/CAS DTCs after an emulator run.
4. **CC 717 (GWS) identity.** It fired once per run with no clear trigger. Make sure it isn't a GWS fault or interlock that would bite in a real drive (KOMBI photo plus GWS DTC read).
5. **GWS unlock vs emulator brake refusal** is not separable on CAN. Run an annotated test with notes of when the lever physically would not move.

**Should fix / check, not gear-critical:**
- FlexRay 0xE6 0x92 at engine stop arrives +730 ms late; it should be +19 ms (checker FAIL).
- CC 18 cause: check the bonnet contact.

**Nice-to-have:**
- 0x0DC P-button timing: FC/FE +16–17 ms late in 10a only.
- The 0x397 3-second duplicates (10a).
- A 0x6F0 → 0x18 / 0x17 diag stub.
- 0x197 B3 value D2 (seen once after the P button: D5 → D2 → C0 in 10a).
- The 0x2ED/0x2DC d2 02→03 timing after pawl.
- A PT-CAN2 capture (needs a second CAN adapter).
