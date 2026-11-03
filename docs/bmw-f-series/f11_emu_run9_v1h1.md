# F11 emulator run 9: firmware v1h.1 (d3ebc30, V2 ignon build), first gear shifts (P→R→P→D→P)

Capture: `f11_emu9_v1h1.7z` → `/workspace/f11-emu-run9/extract/f11_emu9_v1h1.csv.bin` (44 MB), replayed to
`/workspace/f11-emu-run9/replay/` (logger v1h.1, `--replay ... --can --savvy-csv`). Console log `f11_emu9_v1h1.console.log`
(logger v1h.1, firmware `dde-emu-v1h.1`, argv `--emulate --can --can-tx --fake-start cas --start-profile short`).
Checker: `check_emu_capture.py --engine --gear --console ...` → `/workspace/f11-emu-run9/analysis/check_emu_capture_output.txt`.
Scripts: `/workspace/f11-emu-run9/scripts/` (load9, win, glitch/glitch2, cas_p5, missrate, frslots/frcyc, ids9, cc18/cc18b).
Times are seconds from the first CAN row of the capture (logger console time ≈ capture t + 6 s).

Damien's report: "Absolutely everything works." Gear selection works. Parking sensors, radar and power steering work.
The bonnet was up, so the bonnet warnings were expected.

Read-only analysis. Nothing was flashed, pushed or transmitted.

## 0. Summary

| Question | Answer |
|---|---|
| Capture health | **USB drops fixed:** usb_fifo_dropped **0** (run 8: 90 960, 11 %), seq_missing 0, notify/ring 0, tx-log lost 0. 1 056 886 FR frames, 115 104 CAN, 59 956 own Tx (19 IDs). Checker: **1 FAIL** (0x397 spacing, caused by the bug below). |
| Start | Profile **short**. ST_KL D lasted **≈220 ms** (6.17 → 6.39 s; console "START held 218 ms"). CAS accepted it: engine ran 6.18–123.84 s, idle 780 rpm. Console warning: "CAS released START **43 ms** after our running report (real cars: 8–39 ms)". Only 4 ms outside the real range, so harmless. |
| Gears | 4 changes, 0 refusals. **P→R** (lever fwd 2), **R→P** (P button), **P→D** (lever back 2), **D→P** (P button). Brake held for all of them. Lever→0x3FD: N +45 ms / R +108 ms after fwd 2; D +44 ms after back 2; P +915 ms after the P button; pawl 3.04 s later. FR 0xE6 and 0x28 were clean and correct throughout. |
| **New firmware bug** | **Overlay-less PT-CAN bursts.** While the engine ran, ~6 times per second, some of our PT-CAN frames went out **once with stock (engine off / P) content**, then were corrected 1–2 ms later. In total: 666 stale 0x3F9, 462 0x397, 517 0x3B3, 355 0x3FB, **163 0x39A (P + pawl while in R/D)**, 8 0x0DC, 1 0x3FD (P twice while in D: 79.26, 86.36). Run 8: 0. **CAS reacted** (0x12F p5 01→05 "P" within ~2.7 ms of 76/79 false 0x39A P frames). Cause: the core 1 / core 0 seqlock (see §3.4). |
| Bonnet warnings | The bonnet warning itself (CC 18 "Motorhaube offen", sent by CAS 0x5C0) was real. But it **cleared and re-popped 32 times**. **Every clear came within 100 ms of one of our stale "engine not running" 0x3F9 frames** (random chance: 44 %, so a 32/32 run by chance is ~10⁻¹¹). In run 8 it was set once and stayed set. So the "bonnet going mad" was **ours**. |
| Echo misses | **Regression:** 19 265 misses vs 134 747 ok (**12.5 %**; run 8: 0.2–0.4 %), plus 8 944 late. Constant 10–14 % from the first lock to sleep, independent of USB/engine/gears. 91 % "re-armed <40 µs". Slot 0x3D was on the bus in only **~47 %** of its cycles (run 8: 100 %). The car didn't complain. |
| New vs run 8 | No new CAN IDs, no IDs gone. FR: only slot 0xA2 (2 frames at 17.03 s, inside the usual diag burst). **No new CC numbers.** No DTC/fault broadcasts beyond run 8's functional SVK read at 17.0 s (0x6F0 → 0x601/0x62A/0x65E answer; DDE/EGS stay silent by design). Sleep: last frame at ST_KL 7 + **62.31 s** (run 8: +61.09 s). |
| **Verdict** | **Gears work functionally, and the car accepted them.** Two regressions to fix before the next run: (1) the stale-frame bug (easy, core 0 change); (2) echo misses / slot 0x3D presence (needs a bench A/B). |

## 1. Capture health

### 1.1 Counters

| Item | Run 9 (v1h.1) | Run 8 (v1g.2) |
|---|---|---|
| FR frames / CAN / own Tx | 1 056 886 / 115 104 (111 IDs) / 59 956 (19 IDs) | 738 449 / 87 528 / 40 452 |
| usb_fifo_dropped / seq_missing / seq_gaps | **0 / 0 / 0** | 90 960 / 90 960 / 27 764 |
| notify_dropped / ring_overflow | 0 / 0 | 0 / 0 |
| crc_fail (decode rejects) | 1 376 | 487 |
| ts_skipped | 36 391 | – |
| CAN TX fail / fifo-full / TEC / REC | 0 / 0 / 0 / 0 | 0 |
| main loop gap max | 5.8 ms | – |
| tx-log lost | 0 | 0 |

**The 1 KB USB transfers (PICO_FR_USB_FAST) fixed the drops completely.**

### 1.2 Checker (`--engine --gear`)

- PASS: 20 ms IDs at 100 % of slots.
- PASS: E2E on 120 669 static frames, 0 bad.
- PASS: no foreign senders on our IDs.
- PASS: 0x518 last at OFF + 29.75 s; 0x512 at OFF + 45.65 s.
- PASS: 0x397 running median 51 ms; 0x397 post-OFF walk.
- PASS: the three gear checks (agreement 0 of 1996; R/N/D only while running; pawl consistency). **But see §3.4:** the agreement check missed the stale frames. Each flap restarts its 1.2 s settle window, so the flaps hide themselves.
- **FAIL** 0x397 "engine not running: mean spacing 74 ms". Before start it is 362 ms (correct). After OFF it stays at ~60–80 ms until sleep (run 8: back to 360–660 ms). Cause: the same bug. After OFF the overlay sets d0 = 0xD0, and the stale stock frames (d0 = 0xC0) alternate with it: 362 C0 vs 482 D0 frames in 128–187 s. Each change fires an event copy. This is **not** a checker limit this time.

### 1.3 Echo misses (regression)

Console diagnostics (v1h per-miss diag):

| Item | Value |
|---|---|
| ok / miss / late | 134 747 / **19 265** / 8 944 (all "win") |
| miss per slot 0x26/0x28/0x29/0x3D | 0 / 3 899 / 1 675 / **13 691** |
| late per slot | 3 898 / 473 / 372 / 4 182 |
| cause: re-armed <40 µs | **17 590 (91 %)** |
| cause: no-candidate IRQ | 19 446 |
| CRC-near / in-header-ISR | 33 / 0 |
| header ISR max / >10 µs | 189 µs / 61 694 times |
| alarm ISR max | 22 µs |

- The rate was flat at 10–14 % per 5 s from the first lock (before KL15) through start, gears and stop, until sleep. So it does not depend on USB load, the engine or the gear logic being active.
- A typical last miss: `0x3D c15 [re-armed<40us,no-candidate-IRQ] armed by 0x3C 13us before`. Our TX timer is re-armed by the previous slot's header too close to our slot.
- Bus presence measured in the capture (20–50 s):

| Slot | Sent in | Run 9 on the bus | Run 8 |
|---|---|---|---|
| 0x26 | c%4==1 | 82–94 % | 100 % |
| 0x28 | odd cycles | 79–93 % | 93–100 % |
| 0x29 | c%4==3 | 86–97 % | 100 % |
| **0x3D** (wheel torque) | c%4==3 | **31–63 % (~47 %)** | 100 % |

- Prime suspect: the header ISR got longer (max 189 µs). v1h.1 runs engine_cycle → gear_step / gear_fill_out / gear_fill_status / publish inside `dyn_new_cycle` from the FR header ISR. Other v1h changes (USB_FAST, the diag itself) are also candidates.
- Needs a bench A/B (receive-only bench, no car): 7514b68 (v1h) vs d3ebc30 (v1h.1) vs `--no-gear` vs `PICO_FR_USB_FAST=OFF`. Also try the `DDE_EMU_RESCHED_MIN_US=40` guard, and move gear work out of the header ISR.

## 2. Start

| t (s) | Event |
|---|---|
| 5.09 | Brake on (0x173) |
| 5.87 | START pressed (0x12F p5 = 0x85) |
| 5.93 / 6.02 / 6.07 | ST_KL 9 / A / C |
| **6.17** | ST_KL **D** (starter request) |
| D + 6 ms | our E6 0x81 (crank) |
| D + 13 ms | first rpm on 0x0A5 |
| D + 16 ms | crank nibble |
| D + 51 ms | 0x3FB f0 |
| D + 181 ms | our E6 **0x82 (running)** (console "START→82 174 ms") |
| **6.39** | ST_KL **C**: CAS released D, **≈220 ms** after D (console "START held 218 ms"); then A at 6.49 |
| 6.18–123.84 | rpm > 0; idle 780, peak 1059 |
| 122.37 / 122.64 | ST_KL 9 / 8 (stop); our 0x92 +10 ms (PASS) |
| 123.85 | E6 0x80 |
| 124.31 / 124.41 | ST_KL 7 / 6 (KL15 off) |

- **The short profile worked:** the crank dropped from 980 ms (run 8) to ≈220 ms.
- Console: `ENGINE: *** CAS released START 43 ms after our running report (real cars: 8-39 ms) ***`. That is 4 ms over the real window, and CAS still accepted the start.
- 0x592: 24 frames, constant (our MIL off). 0x1B3 bit 4 never set.

## 3. Gears

### 3.1 Lever pushes (brake on for all: 0x173 = 3 from 29.66 to 96.15 s)

| t (s) | Lever (0x197) | Our reaction | Latency | Notes |
|---|---|---|---|---|
| 52.86 / 52.89 | fwd 1 → fwd 2 | 0x3FD N @52.93, **R @53.00**; 0x39A 03/E1 → 02/E0; 0x0DC 25/F4; E6 1/3 → 0/2; 0x28 25/F4 @53.01 | N +45 ms, R +108 ms after fwd 2 | The real EGS gives R +39 ms after fwd 2 (Ian's report), so ours is ~70 ms slower. Console "P → R (brake on) lever forward 2" |
| 53.31 | fwd 1 (in R) | no change | – | correct by our rules (forward from R does nothing), not a refusal |
| 73.80 | P button | 0x39A 01 @73.84; E6 0/1; 0x28 22/FC; **0x3FD P @74.71** | **+915 ms** | real +900 ms |
| 76.84 | – | pawl: E6 2/3, 0x39A E2 | P + 3.04 s (brake still on → 3 s rule) | console "pawl confirmed" |
| 79.08 / 79.11 | back 1 → back 2 | **0x3FD D @79.15**; 0x0DC 24/F4 @79.22; E6 1/3 → 0/5; 0x28 24/F4 | D +44 ms after back 2 | console "P → D (brake on) lever back 2" |
| 79.50 | back 1 (in D) | no gear change | – | the checker's "0x39A 03/E2 +134 ms" here is a **stale frame**, not a reaction |
| 90.09 | P button | E6 0/1 @90.13; **0x3FD P @91.00** | **+915 ms** | |
| 93.13 | – | pawl E6 2/3 | P + 3.04 s | |

Console: changes 4, **refusals 0**, 0x197 ok 6 204, bad 0. The no-brake refusal path was not exercised (brake held throughout).

### 3.2 Did 0x3FD / 0x39A / 0x3F9 / 0xE6 / 0x28 agree?

- **FR 0xE6 and 0x28 (built on core 1): yes, every time,** with no flaps.
- **PT-CAN: yes for the intended content, but with stale single frames.**
  - 0x3FD flashed P at 79.26 and 86.36 while in D.
  - 0x39A showed P + pawl (03/E2) 78× in R (53.2–73.8 s) and 51× in D (79.2–90.1 s).
  - 0x3F9 went to park/stopped 666×.
  - 0x0DC showed P (22/FE) 8× in D (84.1–86.4 s).
  - Each one was corrected 1–2 ms later.

### 3.3 Anything unanswered (GWS/EGS)?

- Nothing on PT-CAN asked for an EGS answer.
- The only request traffic is the functional SVK read at 17.0 s: 0x6F0 `df 03 22 f1 01` (answered by 0x601/0x62A/0x65E). The same burst was in run 8. DDE/EGS stay silent by design (no 0x6xx TX).
- No GWS-related CC (no 169/174/178/203/244/255/302/347/349 etc.).
- Rx IDs that changed right after our gear changes: 0x19F, 0x19A, 0x199 (63/57/55 changes within 300 ms), 0x1B3, 0x0EF, 0x12F. These consumers reacted, they did not complain. 0x12F p5 (CAS "P" bit) followed every gear change, and also the stale frames.

### 3.4 Bug: overlay-less PT-CAN bursts (new in v1h.1)

- **What:** core 0 (`ptcan_emu.c`) takes `dde_engine_snapshot()` once per main-loop pass. `engine_read_g` retries the seqlock 8 times and gives up while the core 1 writer is odd. Then `core.eng = NULL`, `pte_next` builds the **stock template** (engine off, P), and the CYC_EVENT IDs (0x3F9 1 ms, 0x39A 20 ms, 0x3FD gear event) fire at once on the content change. The next pass gets a good snapshot, and the correct frame follows 1–2 ms later.
- **Why now:** v1h.1 put `gear_fill_status(&gear,&eng_pub_gear)` *inside* the write window (dde_emu.c ~899–935), which makes the window longer.
- **Rate:** 5.75 stale 0x3F9 per second while running (max 13/s); ~6/s for 0x397 after OFF. Run 8: 0.
- **Example:** 84.1224 s in D, in one burst: 0x0DC 22/FE (P), 0x0F3 d2 00, 0x3F9 `80 64 3d 3d 23 47` (stopped/P), 0x39A `03 e2` (P + pawl). All were corrected within 2 ms.
- **Effect on the car:**
  - CAS 0x12F p5 went 01→05 ("P") within median 2.7 ms (1.8–82 ms) of **76 of 79** false 0x39A P frames.
  - CAS bonnet CC 18 cleared and re-popped 32× (§3.5).
  - The 0x397 FAIL.
  - Nothing else was visible to Damien.
- **Fix (not implemented):**
  1. Compute `gear_fill_status` into a local *before* the odd write, so the window holds only a memcpy.
  2. On core 0, keep the last good snapshot when a read fails, provided it is < 50 ms old. Never fall back to the stock template while the engine is active.
  3. Optionally use a longer bounded retry.
  4. Checker: add a "stale single-frame reversion" count (payload A→B→A within 20 ms) for 0x3F9/0x39A/0x3FD/0x0DC/0x397, and make it FAIL above 0.

### 3.5 Check-control messages (vs run 8)

Full table: `/workspace/f11-emu-run9/analysis/check_control_run8_vs_run9.csv`. **No new CC numbers.**

| Sender | CC | Meaning | Run 8 | Run 9 |
|---|---|---|---|---|
| **0x5C0 CAS** | **18** | **Bonnet open / "Motorhaube! Vorsichtig anhalten"** | set at D release, refreshed every 10 s, cleared once (8 set / 1 clear) | **ON 34 / off 33 times** (first ON 6.45, last off 122.45) |
| 0x5C0 CAS | 14 | Door open (front right) | 8 set / 1 clear | 2 set / 2 clear, 13.75–15.55 (door opened and shut) |
| 0x5C0 CAS | 148 | Brake-light control | 1 | 1 (2.55, at wake) |
| 0x592 (us) | 34 | Emission warning lamp | 16 | 24 (constant, as designed) |
| 0x590 | 485 | (not in public CC lists) | 10 | 17 (refresh ~6.3 s, scales with run length) |
| 0x581 KOMBI/ACSM | 77 / 181 | seat-belt symbol / airbag lamp (bulb check) | 9 / 2 | 10 / 2 |
| 0x5F2 lights | 120 / 256 / 378 | low beam L failed / headlight range control / lights | ~30 each | ~37 each (pre-existing) |
| 0x5A9 / 0x5B8 | 36, 53, 57, 184, 452, 654 / 45, 245 | DSC/EMF/EHC one-shots at wake | 1 each | 1 each |

**Bonnet:**
- CC 18 is "Motorhaube offen" (bimmerguide CC table). With the bonnet up, one CC 18 was expected, and run 8 had exactly one.
- In run 9 the message cleared and re-popped 32 times. **All 32 clears came ≤100 ms after one of our stale "engine stopped" 0x3F9 frames** (random baseline 44 %). CAS picks the bonnet message variant from engine state, so every false "stopped" frame made it re-evaluate and re-raise the warning.
- **The bonnet warning is expected. Its repetition was caused by us,** and it goes away with the §3.4 fix.

## 4. New vs run 8

- CAN IDs: none new, none gone (`/workspace/f11-emu-run9/analysis/can_id_diff_run8_vs_run9.csv`).
- FR: slot 0xA2, 2 frames at 17.03 s, inside the diag burst. No others.
- Faults / DTC broadcasts:
  - No new CC numbers.
  - No fault states. CAN TX fail 0, TEC/REC 0.
  - The 0x6xx traffic is only the 17.0 s functional SVK read (0x6F0 → 0x601, 0x62A, 0x65E), as in run 8.
- Sleep: last CAN Rx 186.62 s, last FR 186.65 s → **ST_KL 7 + 62.31 s** (run 8: +61.09 s; run 7: +52.63 s). Same behaviour as run 8.
- 0x397 after OFF: ~60–80 ms instead of ~400 ms. This is the same bug, §3.4.

## 5. To do

1. **Fix the stale-snapshot bug** (§3.4: gear status outside the write window, plus keep the last good snapshot on core 0). Then add the stale-reversion check to `check_emu_capture.py`.
2. **Echo-miss bench A/B** (7514b68 / d3ebc30 / `--no-gear` / `PICO_FR_USB_FAST=OFF`, with the resched guard). Target < 0.5 % misses and 0x3D on the bus at 100 %. Consider moving gear_step / publish out of the header ISR.
3. P→R: R comes ~70 ms later than the real EGS (+108 vs +39 ms after fwd 2). Review the 40 ms act + N-display path.
4. Not tested yet: a lever push without the brake (refusal path), and N via the lever.
5. CC 485 (0x590): identify it (present in run 8 too).
