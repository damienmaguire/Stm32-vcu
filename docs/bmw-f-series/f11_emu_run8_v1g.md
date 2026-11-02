# F11 emulator run 8: firmware v1g.2 (895fe20), first fake engine start (real engine still fitted, DDE removed, EGS fuse pulled)

**Inputs:** `/workspace/f11-emu-run8/input/f11_emu8_v1g.7z` (→ `/workspace/f11-emu-run8/f11_emu8_v1g.csv.bin`) and `input/f11_emu8_v1g.console.log`
**Logger/firmware:** banner `fr_ts_logger v1g.2`, `firmware version: dde-emu-v1g.2`, argv `--emulate --can --can-tx --fake-start cas`
**Replay:** `fr_ts_logger.py --replay … --can --savvy-csv` → `/workspace/f11-emu-run8/replay/` (`f11_emu8_v1g.csv`, `.csv.can.csv`, `.csv.savvy.csv`, `.csv.console.log`). This was done early and handed to Fred/Ian.
**Checker:** `check_emu_capture.py --engine --console …`, output in `/workspace/f11-emu-run8/check_emu_capture_output.txt`
**Scripts:** `/workspace/f11-emu-run8/scripts/` (`load8.py`, `usbbytes.py`, `start8.py`, `ids8.py`, `dip8.py`, `dipfr8.py`, `miss8.py`)
**CSVs:** `usb_drop_per_second_run8.csv`, `usb_bytes_per_second_run8.csv`, `can_id_diff_run7_vs_run8.csv`, `check_control_run7_vs_run8.csv` (all in `/workspace/f11-emu-run8/`)

**How times are given:** *t* is seconds from the first CAN frame (T0 = device 45 086 633 µs). Add **≈ 9.4 s** for console time. PT-CAN times are authoritative: CAN records are sent to USB before FR records, and none were lost. FlexRay has USB gaps from t ≈ 7 s (§1.3).

Nothing was flashed, pushed or sent on any bus during this analysis.

---

## 0. Summary

| Question | Answer |
|---|---|
| Did the fake start work? | **Yes.** Brake → START → CAS went 6 → 9 → A → C → **D (KL50 request)** in 0.22 s. Our engine went crank → run, and CAS released D after **0.980 s**. rpm settled at 780 (the dash read ~750). The engine ran in emulation from 7.3 s to 80.0 s and stopped on the second START press. |
| What cranked the real starter, and for how long? | **CAS, through its KL50/ST_KL D state.** D lasted from 6.3633 to 7.3431 s (PT-CAN 0x12F). CAS dropped D in its first 0x12F frame after we sent "running" (E6 b3 = 0x82) plus start-ready = 0. **The crank length is set by our own start profile** (0x82 at D + 0.946 s). |
| Immobiliser? | The ZV never asks the car for an immobiliser exchange, and in testing the car never asked for one, so nothing is needed. |
| Capture health | Logger and firmware v1g.2: OK. Tx rows present (40 452, 19 IDs). **usb_fifo_dropped = 90 960 (11 %)**, notify_dropped 0, ring_overflow 0. Checker: 2 FAILs, USB drops (real) and 0x397 spacing (a checker limit, see §2). |
| Emulator health | 20 ms IDs at 100 % of slots. CAN TX fail 0, fifo-full 0, TEC/REC 0. No fault state. **New: echo miss = 343 (run 7: 1) and late/short = 15 (run 7: 0)**, spread evenly over the whole run. |
| Sleep | Last frame at ST_KL 7 + 61.09 s (CAN) / + 61.15 s (FR). Run 7: + 52.63 s. Every vehicle node shut down about 8.4 s later, in the same order. Our 0x518/0x512 stopped on schedule. Most likely an effect of the engine-run shutdown, not of us (no real-DDE engine-off baseline exists). |
| **Verdict** | **v1g.2 PASSES the start function.** Before the next run, and certainly before the engine comes out: (1) shorten the crank in software, and/or disable the starter in hardware while the real engine is fitted; (2) USB drops are reduced but not gone, so build with `PICO_FR_USB_FAST`; (3) find out why echo misses rose. |

---

## 1. Capture health

### 1.1 Counters

| Item | Value |
|---|---|
| frames (FR) / CAN / own-Tx | 738 449 / 87 528 (111 IDs) / 40 452 (19 IDs) |
| device valid / usb_fifo_dropped | 829 409 / **90 960** (seq_missing 90 960, seq_gaps 27 764) |
| notify_dropped / ring_overflow / len_mismatch / parse_fail | 0 / 0 / 0 / 0 |
| crc_fail (decode rejects) | 487 (run 7: 759 in 1.45 M; similar rate) |
| clock_anomalies / bytes_skipped | 0 / 0 |
| tx-log lost | 0 |

### 1.2 Checker (`--engine`)

- PASS: logger v1g.2, firmware dde-emu-v1g.2.
- PASS: 20 ms IDs 0x0A5/0D9/0DC/0F3: 7160 frames each, 100 % of slots, max gap 21–40 ms.
- PASS: 0x08F, 0x2C4, 0x281, 0x3FD: 100 %.
- PASS: no foreign senders of our IDs. E2E: 108 666 static frames, 0 bad.
- PASS: 0x397 after OFF.
- PASS (engine): start at 6.37 s (ST_KL D). First rpm +103 ms, E6 81 +101 ms, E6 82 +941 ms, D released +1000 ms; 82 came before the release. Idle 780 (30 0C). rpm > 0 from 6.48 to 81.48 s, max 1059.
- 0x518 last frame OFF + 30.18 s (real DDE 29.93). 0x512 OFF + 46.27 s (real 45.93).
- 0x592: 16 frames, content constant `40 22 00 14 ff ff ff ff` (CC 34, status 0x14).
- **FAIL** usb_fifo_dropped = 90 960. Real (§1.3).
- **FAIL** 0x397 mean spacing 99 ms, with 30 e3 blips. **Checker limit:** v1g sends 0x397 every 60 ms while the engine runs. The checker still uses the 250–600 ms engine-off window. Make it use the 60 ms target while E6 = 82.

### 1.3 USB drops: reduced, not fixed

Per second (`usb_drop_per_second_run8.csv`, `usb_bytes_per_second_run8.csv`):

| Phase | Drops/s | Offered | Delivered |
|---|---|---|---|
| t < 7 s (wake, KL15 just on) | 0 | up to 236 kB/s | = offered |
| 7–81 s (KL15 on, engine running) | 850–1100 | 243–248 kB/s | **mean 215 kB/s (204–235)** |
| 82–127 s (after OFF, nodes still up) | 300–600 | | |
| ≥ 133 s | 0 | | |

Comparison:
- Run 6 (v1e): 0 drops at up to **242.5 kB/s** (mean 231).
- Run 7 (v1f): dropped while delivering only ~202 kB/s.
- Run 8: no drops up to 236 kB/s.

So the v1g.2 TEF fix did recover most of what v1f lost. The cause that remains is the known **64-byte-per-transfer TinyUSB vendor ceiling** (`CFG_TUD_VENDOR_EPSIZE 64`, one transfer re-armed per `tud_task()`; see `f11_schedule_run4.md` Q1). With KL15 on, the engine running and own-Tx logging (~290 rec/s, ~6 kB/s), the offered load is 243–248 kB/s, above the ceiling. Delivered throughput also sags below the no-drop rate once it is overloaded.

**Fix:** build with `-DPICO_FR_USB_FAST=1` (1 KB transfers, 16 KB TX buffer; commit acf675e, never tested on hardware). The 64·N bulk-OUT caveat does not apply: the logger sends no bulk OUT, only control requests. Bench-test first.

Impact on this analysis: PT-CAN is complete. FlexRay has gaps from t ≈ 7 s. The D onset (6.36 s) fell in a drop-free second, and D release is timed from PT-CAN.

---

## 2. Start sequence timeline (PT-CAN unless marked FR)

| t (s) | Event |
|---|---|
| 5.1727 | Brake on: 0x12F d6&3 = 3 (FR 0x74 p6 = 3 at 5.18; 0x173 d7&3 = 3 at 5.1963) |
| 5.9729 | START pressed: 0x12F d5 = 0x85 (FR 0x74 p5 = 0x85 at 5.98). Released 6.09 |
| 6.0928 | 0x12F d4 f1/f2 → f4. 0x12F rate goes from 100 ms to ~20–50 ms (20 ms from 6.30) |
| 6.1433 | ST_KL **9** (FR 0x74: 6.18) |
| 6.2079 | ST_KL **A** (KL15 on) (FR 6.21) |
| 6.2632 | ST_KL **C** (FR 6.27) |
| **6.3633** | ST_KL **D = starter request / KL50** (FR 6.37). **Real starter cranks** |
| 6.4424 | Our 0x0D9 CounterHi 1 → 3 (back to 1 at 6.5424) |
| 6.4431 | 0x12F d2 high nibble → 8 (0x8D) |
| 6.4692 / FR 6.4727 | Our E6 b3 0x80 → **0x81 (crank)**; start-ready (0x3F9 d6 / E6 b8 hi nibble) 2 → 1; E6 b4 0x64 → 0x65 |
| 6.4795 | First rpm on 0x0A5: 131. Crank 123–201 rpm until 7.06 |
| 7.08 → 7.30 | Run-up: ≥ 300 at 7.119, ≥ 600 at 7.199, ≥ 750 at 7.259, ≥ 900 at 7.299 |
| 7.3093 / FR 7.3126 | Our E6 b3 → **0x82 (running)**, b4 0x66 |
| 7.3233 | 0x12F still D (14 ms after 82) |
| 7.3342 / FR 7.3376 | Our **start-ready → 0** |
| **7.3431** | ST_KL **C: starter request released** (9 ms after ready-0, 34 ms after 82). FR 0x74 C at 7.37 |
| 7.419 | rpm peak 1059 (flare), then a smooth decay to 800 at ~10.5 s and 780 by ~13 s. Idle 779.75–780.75 rpm 20–79 s |
| 7.5232 | ST_KL **A** (0x12F d2 = 0x8A). E6 b4 0x66 → 0x26 at 7.5645 |
| 7.853 | Brake off (0x173 at 7.876) |
| 80.02 | START: ST_KL 9 (stop). Our E6 0x92 at 80.32, 8 at 80.29, rpm 0 at 81.48, E6 0x80 at 81.52 |
| 82.62 / 82.72 | START again: ST_KL 7 → 6 (KL15 off) |

- **D length: 0.980 s** (PT-CAN 6.3633 → 7.3431; FR 6.37 → 7.37). This matches Damien's "about a second".
- CAS went to C/D 0.39 s after the button. The only 0x592 frames are our own CC 34 (MIL off) frames.
- **IDs first appearing around the start** (5.5–9 s): CAN 0x0D7, 0x1B3, 0x226, 0x246, 0x26A, 0x2A5, 0x2C3 (6.354), 0x394 (6.392), 0x5B0 (6.538), 0x1F6 (8.24) and others. FR 0x31 (6.36), 0xEA (6.40), 0x10A (6.42), **0x13B (7.348)**.
  - The same IDs appear at KL15-on in run 7, at the same offsets: FR 0x31 at KL15 + 0.15 s, 0x13B at KL15 + 1.14 s.
  - None is new versus run 7, so none is starter-specific.
- **Crank evidence on the bus:** a scan of every PT-CAN and FlexRay byte for "steady before, different during 6.55–7.25 s, back after" found **no battery-voltage dip or KL50 feedback signal**. The only hits were single-frame noise (0x2A7, 0x36E). Crank timing therefore rests on ST_KL D plus Damien's observation.

## 3. What made the starter crank, and how to shorten it

**Mechanism:** with brake on and START pressed, CAS asks for the start (ST_KL D). With the DDE removed and the EGS fuse pulled, CAS still drove KL50 to the real starter. Every condition it checks on the bus must therefore have come from our emulated frames or from modules still fitted. If there is a park/neutral interlock over the bus, our emulated frames or the selector lever satisfied it; worth confirming. CAS **holds D until the engine reports running**, then drops it within one 0x12F frame (20 ms).

**Our side:** the v1g.2 start profile copies a real start:
- crank state 0x81 at +0.10 s;
- crank rpm ~150 for 0.6 s;
- run-up;
- 0x82 at +0.946 s;
- start-ready 0 at +0.971 s.

CAS released at +0.980 s. **So the crank lasts as long as our profile tells it to.**

Which signal CAS keys on is not certain from this run:
- The 0x12F frame 14 ms after 0x82 still showed D. The next one (9 ms after start-ready 0) showed C.
- The real DDE capture showed release 8 ms after 0x82.
- Either CAS takes 14–34 ms to react to 0x82, or it waits for start-ready 0.
- **Send both together.**

Recommendations (not implemented):
1. **While the real engine is fitted, don't rely on software:** remove the starter relay/fuse or unplug KL50 at the starter for further emulator tests. The faulty engine was spun for about 1 s this time.
2. **Short profile (preferred first step):** on D, send 0x81 and the first rpm in the next E6/0x0A5 slot (~10–20 ms). Ramp rpm quickly (e.g. 150 → 780 in ~100 ms). Send **0x82 + start-ready 0 together** at ~+150–200 ms.
   - Expected D: ~0.2 s, including CAS reaction.
   - The event order stays the same as a real start, which lowers the risk of CAS plausibility faults.
3. **Immediate profile (test with starter disabled):** send 0x82 + ready 0 + idle rpm in the first cycle after D. Expected D ≈ 30–50 ms, which is about the solenoid pull-in time.
   - Risk: CAS may have a minimum-crank or rpm-plausibility check and could retry, abort or log a fault.
   - Check CAS fault memory afterwards.
4. **Do not** report "running" before START: at 80.02 s CAS treated START with the engine "running" as a stop request (ST_KL 9).
5. **Once the engine is out:** KL50 will drive nothing, or an open circuit if the starter wiring is removed. Read CAS DTCs after a few starts. If CAS flags open-load on KL50/starter relay, add a dummy load or keep the relay.
6. **Separation test (optional, starter disabled):** send start-ready 0 first and 0x82 100 ms later (and the reverse) to learn which signal releases D.

## 4. Emulator health

### Rates and TX

| Item | Value |
|---|---|
| 0x0A5/0D9/0DC/0F3 | 7160 each, 100 % slots |
| 0x08F | 4774 |
| 0x3FD/0x2C4/0x281 | 1431–1432, 100 % |
| CAN TX | 40 452 sent, fifo-full 0, fail 0, TEC 0, REC 0 |
| Loop timing | loop-gap max 5.3 ms, > 20 ms: 0, late max 4.2 ms, resync 0 |
| 0x518 / 0x512 stop | 110.19 s (OFF + 30.17 s) / 126.29 s (OFF + 46.27 s). Real: 29.93 / 45.93 |
| 0x397 | every ~60 ms while running (by design); after OFF OK per checker |
| 0x592 (our MIL/CC 34) | 16 frames from 6.31 s every ~5.1 s, always `40 22 00 14`. MIL never set. |

### FlexRay emulation (console `emu:`)

| Item | Value |
|---|---|
| echo ok | 125 252 |
| **echo miss** | **343 (0.27 %)** |
| late | 19 (busy 0, fire 0, **short 15**, win 4, missed 0) |
| skip FB7 | 4 |
| lock_losses | 1 (at bus sleep, normal) |
| foreign / faults | 0 / none (state ACTIVE until sleep) |

- **Regression vs run 7** (echo miss 1, late short 0).
- Misses grow at ~2–3 per second from t ≈ 1 s (before ignition) until sleep. They do not depend on KL15, the start, or engine running.
- The checker does not fail on this, and the car showed no symptom. It still needs a bench comparison of v1f against v1g.2 echo counters with identical bus load.
- Candidates:
  - the v1g engine-state code paths;
  - extra core-1 / SPI time from TEF and own-Tx logging;
  - the USB drain under load.
- An attempt to locate missed slots in the capture (`miss8.py`) was inconclusive, because the logger shows our own slots as null in most cycles.

### Check-control (`check_control_run7_vs_run8.csv`)

- **New in run 8** (Rx, not from us):
  - 0x5C0 (CAS/FEM) **CC 18**: set 7.35 s, at D release.
  - 0x5C0 **CC 14**: set 7.85 s, at brake release.
  - Both repeat every 10 s and both clear at 80.05 / 80.15 s (engine stop).
  - 0x590 **CC 485**: set 28.42 s, repeats every 6 s, cleared 80.35 s.
  - All three follow the "engine running" state, so they are probably start/run related. Damien saw only lamp-failure warnings, so these may be silent or informational messages.
  - **Look up CC 14, 18 and 485 in ISTA or the CC list.**
- Our 0x592 CC 34 is only "new" because run 7 had no Tx rows. The console shows 18 such frames in run 7 too.
- Gone versus run 7: CC 66 / 652 / 771. Those were dead-key (wake 1) messages.
- New CAN IDs: only diagnostic 0x601/0x65E at 17.5 s (with 0x62A/0x6F0; tester/diag burst).
- New FR IDs: single frames of 0x90/93/9A/AB/B4/B7/C6/CF/D1 at ~17.5 s, the same diag burst.

### Sleep

- Bus sleep came at ST_KL 7 (82.62 s) + 61.09 s CAN / + 61.15 s FR, or first OFF press + 63.69 s.
- Run 7: + 52.63 / + 52.72 s.
- The shutdown order of all vehicle IDs is identical to run 7. Every node is ~8.4 s later (e.g. 0x2F8 + 52.77 vs + 46.01, 0x24F + 48.84 vs + 46.92).
- Our frames stopped with everyone else's. 0x512 stopped ~17 s before sleep; in run 7 it was ~6.6 s before.
- Most likely the vehicle's normal longer after-run following an engine run. There is no real-DDE engine-off baseline to confirm it.

## 5. Comparison with run 7 (v1f)

| Item | Run 7 (v1f) | Run 8 (v1g.2) |
|---|---|---|
| Logger / Tx rows | v1e logger, 0 Tx rows | v1g.2, 40 452 Tx rows |
| Engine start | none (KL15 only) | D 0.980 s, crank → 780 rpm idle |
| 20 ms frames | 50.0/s by counters | 100 % of slots, verified frame by frame |
| USB drops | 187 933 (13 %), ~202 kB/s delivered | **90 960 (11 %)**, ~215 kB/s delivered, 0 up to 236 kB/s |
| Echo miss / late | 1 / 31 (all win) | **343 / 19 (15 short)** |
| crc_fail | 759 / 1.45 M | 487 / 0.83 M |
| CC messages | dead-key CC 66/652 | **CC 14, 18 (0x5C0), 485 (0x590)** while running |
| Sleep after KL15 off | + 52.63 s | **+ 61.09 s** (all nodes later) |

## 6. To do (v1h / next run)

1. Disable the starter in hardware for further tests while the engine is fitted.
2. Implement the short start profile (§3, item 2), then test the immediate profile with the starter disabled.
3. Build with `PICO_FR_USB_FAST=1`, then bench-test for `usb_fifo_dropped = 0` at ≥ 250 kB/s.
4. Bench comparison of echo miss and late/short between v1f and v1g.2.
5. Checker: use the 0x397 60 ms target while E6 = 82.
6. Identify CC 14/18/485. Read CAS/ZGM DTCs after the run.
