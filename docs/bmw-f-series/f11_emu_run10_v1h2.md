# F11 emulator run 10: firmware v1h.2 (bc62e95, V2 ignon build), runs 10a + 10b

Captures: `f11_emu10_v1h2.7z` (run 10a, 194.3 s) and `f11_emu10_v1h2_2.7z` (run 10b, 148.1 s).
- Extracted to `/workspace/f11-emu-run10/extract/`.
- Replayed with the **v1h.2** logger (`fr_ts_logger.py --replay … --can --savvy-csv …`) into `/workspace/f11-emu-run10/replay/`.

Checker: `check_emu_capture.py --engine --gear --flicker --console …` from the v1h.2 dist. Outputs are in `/workspace/f11-emu-run10/analysis/check_f11_emu10_v1h2{,_2}.txt`.

Scripts are in `/workspace/f11-emu-run10/scripts/`:

| Script | What it does |
|---|---|
| load10 | loads the runs |
| lever | 0x197 changes and gaps |
| e6 | dumps the 0xE6 / 0x74 slots |
| slots10, gaps10 | own-slot presence on the bus |
| events10 | FR diagnostic and DTC frames |
| cc10 | check-control messages |
| brake10 | brake, lever and 0x5DE timeline |

The script outputs are in `analysis/` (`events_run10.txt`, `cc_run10_vs_run9.txt`, `brake_lever_run10.txt`).

**Times** are seconds from the first CAN row of each capture. Logger console time = capture t + ~16.6 s in 10a and + ~19.7 s in 10b.

**10b was not a cold start.** The board was not power-cycled between the runs (ignon build), so the 10b firmware counters continue from 10a. 10b-only figures below are the 10b counters minus the 10a counters.

**Damien's report:** "From what I could tell the car behaved as OEM. Shifts without brake and/or without the side button on the stick were refused. Car thought it was running."

Read-only analysis. Nothing was flashed, pushed or transmitted.

## 0. Summary

| Question | Run 10a | Run 10b | Run 9 (v1h.1) |
|---|---|---|---|
| USB drops (usb_fifo / seq_missing / gaps) | **0 / 0 / 0** | **0 / 0 / 0** | 0 / 0 / 0 |
| Echo misses (firmware) | **55 / 169 336 = 0.032 %** | **37 / 129 315 = 0.029 %** | 19 265 / 154 012 = 12.5 % |
| Misses, slots 26/28/29/3D | 0 / 51 / 3 / 1 | 0 / 30 / 5 / 2 | 0 / 3 899 / 1 675 / 13 691 |
| Late frames, slots 26/28/29/3D | 0 / 0 / 0 / 0 | 0 / 1 / 0 / 0 | 3 898 / 473 / 372 / 4 182 |
| Presence on the bus (capture), 0x26 / 0x28 / 0x29 / 0x3D | 100 / 99.87 / 99.98 / 99.99 % | 100 / 99.86 / 99.96 / 99.97 % | 87.6 / 85.6 / 93.3 / **41.0 %** |
| 0xE6 frames / max gap | 209 / 1.04 s (1 s grid) | 159 / 1.04 s | 187 / **3.75 s** (9 gaps > 1.05 s) |
| Snapshot read fails / background step max | **not recorded** (logger v1h.1, see §1) | not recorded | n/a |
| Flickers (0x39A / 0x3FD / 0x3F9 / 0x0DC / 0x397) | **0 / 0 / 0 / 0 / 0** | **0 / 0 / 0 / 0 / 0** | 163 stale 0x39A, 666 0x3F9, … |
| DTC events on FR | **none** | **none** | ICM D0156D ×11, EPS D51744 ×3 |
| Check-controls | CC 18 set once, refreshed every 10 s, cleared at OFF (no flapping). New: GWS CC 717 ×1 | same; GWS CC 717 ×1 | CC 18 flapped 34 on / 33 off |
| Start | ST_KL D 199 ms; first rpm +3 ms; 82 at +176 ms; CAS released 24 ms after our running report; **accepted** | D 198 ms; rpm +13 ms; 82 at +186 ms; CAS +23 ms; **accepted** | D 218 ms, CAS +43 ms |
| Gears | 8 changes, 0 refusals (P→D, D→N, N held 7.3 s, N→D, D→N, N→R, R→N, N→D, P button) | 5 changes, 13 refusals (all real brake-off pushes) | 4 changes, 0 refusals |
| Sleep after ST_KL 7 | +79.9 s = last door event + 56.8 s | +71.2 s = last door event + 56.5 s | +62.3 s = last door event + 56.4 s |
| Checker | 2 FAIL: logger version (cosmetic) and 0xE6 0x92 at +728 ms | 2 FAIL: the same, 0x92 at +746 ms | 1 FAIL |

**Verdict:** v1h.2 is **good enough to call done for now**.
- The two run 9 regressions are fixed:
  - stale PT-CAN frames: 0 flickers;
  - echo misses: 12.5 % → 0.03 %, and 0x3D is on the bus ~100 % of the time.
- There were no DTC broadcasts at all, and the bonnet warning no longer flaps.
- Brake gating works, the GWS's own unlock-button gating works, and the car accepted every start.

Three small v1h.2 bugs were found (§4). None was visible to Damien and none changed what the car did. They are worth a v1h.3 later, not now.

## 1. Logger v1h.1 against firmware v1h.2 (VERSION MISMATCH)

I diffed `fr_ts_logger.py` / `frts.py` between the v1h and v1h2 dists. Only three things differ:
1. `LOGGER_VERSION`.
2. `EMU_DIAG_STRUCT`: the last 4 bytes of the emu-diag reply. v1h.1 decodes them as `I reserved`; v1h.2 decodes them as `HH snap_fail, bg_max_us`.
3. `GEAR_STAGES`: v1h.2 adds stage 5 "P released". v1h.1 prints it as a raw number.

The USB protocol, all control requests (emulate, can-tx, fake-start, start-profile, gear), record parsing and the .bin/.csv writers are byte-identical.

**What the mismatch does not affect:**
- The capture itself (FR, CAN and own-Tx records). Re-replaying the .bin with the v1h.2 logger gives the same data. seq_missing is 0.
- The commands sent to the firmware, so the car saw normal v1h.2 behaviour.
- Every console counter except the two below.

**What it does affect:**
- The two new v1h.2 diagnostics, **snapshot read fails** and **background step max**, were never printed. They live only in the live diag reply and not in the .bin, so they **cannot be recovered for run 10**.
- An indirect substitute: the checker's flicker test (which catches what a failed snapshot read would produce) is 0 in both runs.
- The checker FAIL "logger warned VERSION MISMATCH" is cosmetic.

## 2. Capture health and echo

| Item | 10a | 10b (capture) | Run 9 |
|---|---|---|---|
| FR / CAN / own Tx (IDs) | 1 124 423 / 119 038 / 54 823 (19) | 855 289 / 90 449 / 41 581 (19) | 1 056 886 / 115 104 / 59 956 |
| usb_fifo_dropped / notify / ring / seq_missing | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0 |
| CAN TX fail / fifo-full / TEC / REC / tx-log lost | 0 / 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 / 0 | 0 |
| E2E | 154 949 frames, 0 bad | 118 374, 0 bad | 0 bad |
| hdr ISR max | 92 µs | 94 µs | 189 µs |
| alarm ISR max | 19 µs | 19 µs | 22 µs |
| re-armed < 40 µs misses | **0** | **0** | 17 590 |
| last-miss cause | no-candidate-IRQ, prev-frame-on-wire (0x28, armed by 0x21 211 µs before) | same | re-armed < 40 µs |

- The remaining misses (~0.03 %) are a different, rare mechanism: 0x28 right after a frame on the wire. They are not a problem.
- The header ISR still reaches ~92 µs and is > 10 µs on most cycles. This is headroom to look at some day, not a fault.
- The dynamic skips were tiny: E6 1, FB7 2, E8L9 3 per run.
- **Bus-side check** (`gaps10.py`): 0x26 / 0x29 / 0x3D are expected every 20 ms and 0x28 every 10 ms.
  - Worst gap: 20 ms for 0x26/0x28 and 40 ms (one missed frame) for 0x29/0x3D.
  - Run 9 had gaps of 120 ms (0x26) and 480 ms (0x3D).

## 3. Car behaviour

### 3.1 Faults / DTCs

The FR diagnostic slots (0x90 0x93 0x9A 0xA2 0xAB 0xB4 0xB7 0xBE 0xCF) carry only the usual wake identification read:
- F101 multi-frame answers from EPS, ICM and DSC;
- DSC 7F 22 78 (response pending);
- one tester → DDE 22 F101, which we leave unanswered by design.

This happened at 16.1 s (10a) and 17.7 s (10b), as in run 9.

**No unsolicited DTC event (62 17 04 …) in either run.** Run 9 had ICM D0156D ×11 (caused by our missing 0x3D) and EPS D51744 ×3 (E6 gaps > 2 s). Both causes are gone: 0x3D is at ~100 % and the E6 max gap is 1.04 s.

PT-CAN 0x6xx traffic is the same functional SVK read as run 9 (0x6F0 → 0x601/0x62A/0x65E).

### 3.2 Check-controls (`cc_run10_vs_run9.txt`)

**CC 18 (CAS 0x5C0, "bonnet open"):**
- Set once just after the start (5.95 s / 7.57 s), re-broadcast every 10 s, and cleared once at OFF.
- This is exactly the run 8 / real-DDE pattern. The real-engine captures 4log_3krev and 4log_r10md10m show the same CC 18 while running.
- **The run 9 flapping (34 on / 33 off) is gone.**
- However, it is **still raised**, although Damien says the bonnet was closed.
  - The car's own door/lid frame 0x2FC is byte-identical in run 9 (bonnet confirmed up) and run 10 (`81 0x 04 00…`).
  - So CAS still reads its bonnet contact as open. We don't send anything bonnet-related.
  - Either the cluster didn't show it, or the bonnet switch/latch contact needs checking. Worth asking Damien whether "bonnet open" was on the display.

**Other CCs:**
- CC 14 (door open, 10a 6.45–10.75 s).
- CC 148 (one clear at wake).
- CC 77/181 (belt/airbag), 0x5F2 lights 120/256/378, 0x590 CC 485, and the DSC/EHC wake one-shots. All of these as in run 9.
- Our 0x592 content never changed (checker).

**New: CC 717 from 0x5DE**, once per run:

| Run | Set | Cleared |
|---|---|---|
| 10a | 46.18 s, in N, brake on | 47.39 s |
| 10b | 29.00 s, in P, brake off | 29.57 s |

- 0x5DE fits the 0x580 + diag-address pattern for the gear selector (GWS 0x5E): 0x5C0 = CAS 0x40, 0x5A9 = DSC 0x29, 0x592 = DDE 0x12. Treat this as **likely** rather than confirmed.
- There was no 0x197 change at either moment.
- CC 717 is not in run 8, run 9 or the real captures, and not in the public CC lists I found.
- **Likely** it is the GWS's own hint for a push without the unlock button (Damien tested exactly that). It is not caused by us. Worth asking Damien what text he saw.

### 3.3 Start

| Item | 10a | 10b |
|---|---|---|
| ST_KL D at | 5.72 s | 7.32 s |
| First rpm | +3 ms | +13 ms |
| E6 81 | +1 ms | – |
| E6 82 | +176 ms | +186 ms |
| ST_KL D released | +200 ms | +200 ms |
| Console: START held | 199 ms | 198 ms |
| Console: CAS released after our running report | 24 ms | 23 ms (real 8–39 ms) |
| Engine ran | 5.72–114.1 s | 7.3–77.7 s |

CAS accepted both starts. Idle was 780 rpm, and 0x397 spacing, E2E and 0x3FB were all PASS.

### 3.4 Gears (lever 0x197: B2 high nibble 0 = centre, 1 = fwd1, 2 = fwd2, 3 = back1, 4 = back2; B3 C0 = released, D5 = P button)

**Run 10a** (brake on for every change, 0 refusals):

| Time | Change | Latency |
|---|---|---|
| 28.95 s | P→D (back1) | +42 ms |
| 32.58 s | D→N (fwd1) | – |
| 32.58–39.84 s | **N held 7.3 s** | – |
| 39.84 s | N→D | – |
| 44.76 s | D→N | – |
| 49.38 s | N→R (fwd1 from N) | +42 ms |
| 59.67 s | R→N | – |
| 61.98 s | N→D (back1/back2) | +42 ms from back2 |
| 93.34 s | P button | 0x39A 01 at +25 ms; P at +902 ms (94.25 s) |

- Pawl confirmed at 96.35 s.
- 0x3FD / 0x39A / 0x3F9 / 0xE6 / 0x28 agree throughout, R/N/D only while running, and CAS 0x12F p5 follows (01 at 28.96, 05 at 96.37).
- There are no spurious CAS "P" reactions (run 9 had 76).

**Run 10b:**

| Time | Event |
|---|---|
| 21–36 s | 20 real lever notches from P (fwd1/fwd2 and back1/back2) with the **brake off** (0x173 off 9.78–37.25 s) → 11 refusals, "brake not pressed" |
| 38.05 / 38.11 s | P→N→R: N at fwd1 +42 ms, R at fwd2 +41 ms (run 9: R +108 ms; real EGS +39 ms). N is shown for 60 ms in between (cosmetic) |
| 48.62 s | R→N with brake off: allowed (OEM: N needs no brake) |
| 51.3 s | N→D with brake off: refused (2 more refusals, 13 total) |
| 53.93 s | N→D with brake on: D at +42 ms after back2 |
| 67.35 s | P button → P at +903 ms (68.26 s); pawl at 70.36 s |

**Pawl:**
- In both runs the pawl came from the 3 s timeout (+3.0 s), because the brake was still held: released at 113.23 s / 75.68 s.
- So the "brake released → pawl +340 ms" path is still untested in the car.

**Not tested:** a direct R→D (back2 from R) and D→R.

### 3.5 Side (unlock) button

- 0x197 carries only the notch nibble and the P button. Across all of run 9 / 10a / 10b it has exactly 7 distinct values, with no button bit (low nibble always F). Ian was right.
- **No-button attempts never reached us.** The checker lists every 0x197 notch. Every notch is either a shift, a refusal by us for no brake, or the lever returning to centre. There is no notch the button-less attempts could correspond to.
- So the unlock-button gate is in the GWS itself: it blocks the move or doesn't report it, and probably raised CC 717.
- The brake-off pushes in 10b did produce notches, which we refused. **Both gates worked: the GWS's (button) and ours (brake).**

### 3.6 Sleep

- In all three runs the capture ends abruptly at full traffic rate. That is the bus going to sleep: the 10a console shows "0 frames/s" before Damien stopped the logger.

| Run | ST_KL 7 at | Last frame | Last door event (0x2FC byte 1) | Bus asleep |
|---|---|---|---|---|
| 10a | 114.38 s | +79.9 s | 137.46 s | +56.8 s after it |
| 10b | 76.92 s | +71.2 s | 91.59 s | +56.5 s after it |
| 9 | – | +62.3 s | – | +56.4 s after it |

- **Sleep timing is normal and identical.** The longer OFF→sleep times are just door activity after OFF.
- Our 0x518 / 0x512 stopped at ST_KL 7 +28 / +44 s (10a) and +29 / +45 s (10b), as in run 9.

## 4. v1h.2 bugs found (none visible in the car; v1h.3 material)

**A. Lever/brake timestamp wrap ("lever 0x197 lost" false alarms).**
- **Cause:** `dde_emu_core1_poll` takes `now = time_us_32()` before gear_tick reads the lever/brake mailboxes, which core 0 stamps.
  - A 0x197 frame that arrives in between gives `t > now`.
  - `ms_since()` (dde_gear.c, unsigned `(now - t)/1000`) then wraps to a huge value, so it reads as "> 90 ms stale" and logs "lost", then "back" on the next frame.
- **Counts:** 10a 3, 10b 17, run 9 0. PT-CAN 0x197 has no real gap > 90 ms in 10b; 10a has one 95 ms gap at 137.2 s, after OFF.
- **Risk:** a "lost" clears a pending lever gesture. The same wrap can make `brake_on` read false for one tick, which would give a spurious "brake not pressed".
  - It didn't happen in run 10: every push gave the right result, and every refusal had the brake really off on 0x173.
- **Fix:** signed clamp `int32_t d = now - t; return d < 0 ? 0 : d / 1000;`, or take `now` after reading the mailboxes.

**B. E6 0x92 "stopping" event lost until the 1 s grid (the checker FAIL).**
- **Observed:** 0x92 came +728 ms (10a) / +746 ms (10b) after ST_KL 8. Run 9: +10 ms; real DDE +19 ms.
  - 10a FR: an E6 event at 112.911 s still carried 82/66, and 92/66 only appeared on the grid at 113.626 s.
- **Cause:** engine_publish now runs in the core 1 background. It sets `eng_e6_event`, and fire_slot clears the flag when the E6 goes out (dde_emu.c ~692).
  - If the 92 publish lands after the E6 payload was built but before it fires, the old payload goes out and the flag is cleared. The event is then lost until the next 1 s grid frame.
  - Gear E6 changes could hit the same race. In run 10 they all came out on time.
- **Effect:** EPS/ICM saw "stopping" ~0.7 s late at OFF. There was no DTC and no visible effect.
- **Fix:** clear the flag when the payload is built, or tag the event with a sequence number and keep it pending while the built payload's seq is older than the published seq.

**C. Console gear status falls back to zeros.**
- `dde_gear_fill_status` memsets the status when the bounded seqlock read fails. ev_seq then drops to 0, and the logger printed "(65524 older gear events not shown)" and re-printed 5 old events (10a, logger t = 58.4).
- This is console only; PT-CAN/FR are unaffected.
- **Fix:** keep the last good status, as the PT-CAN snapshot path already does.

## 5. Conclusion

**v1h.2 is good enough to call done for now.** It matches Damien's "behaved as OEM":
- every run 9 problem is fixed;
- there are no faults on the bus;
- the shift gating is correct, and starts are accepted with OEM-like timing.

Worth refining later. All of these are small and none is urgent:
1. v1h.3: fix A (timestamp clamp), B (E6 event race) and C (status cache).
2. Next car run: use the **matching logger** so snapshot read fails and background step max are recorded.
3. Next car run: power-cycle the board between runs.
4. Next car run: test the untested paths:
   - release the brake within 3 s of the P button (pawl via brake release);
   - a direct R→D / D→R.
5. Ask Damien whether "bonnet open" and a gear-selector message (CC 717) appeared on the cluster. Check the bonnet contact if CC 18 shows with the bonnet shut.
6. Optional: header ISR max ~92 µs leaves headroom on the table; the remaining 0x28 "prev-frame-on-wire" misses (~0.1 % of 0x28).
