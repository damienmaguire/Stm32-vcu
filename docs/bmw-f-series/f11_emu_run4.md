# F11 emulator car test 4 (firmware v1d, V2 pico-flexray board): FlexRay emulation + PT-CAN capture

2026-10-31, Europe/Dublin (GMT, UTC+0). Analysed from Damien's archive `01M6969N6Q0FH02NDBVXXH919W.7z`,
extracted to `/workspace/f11-emu-run4/`. Firmware v1d, branch `dde-emulator` (logger 366997d). The real DDE was on the car
with its **own PT-CAN connected** and its FlexRay replaced by the emulator ("ddecan" run). Damien reports a clean dash.
[E] = measured here, [H] = hypothesis. t = s after the first captured FlexRay record.
Scripts: `emu_run4_scripts/` (load4, overview, kl15, passcheck4, ptcan_ids, e2e_dde, twins, brake).
Tables: `f11_emu_run4_ptcan_ids.csv` (PT-CAN inventory, 128 IDs), `f11_emu_run4_dde_ptcan_baseline.csv` (§4).

## 0. What was in the archive

| file | size | notes |
|---|---|---|
| `f11_emu4_ddecan.csv` | 86 035 387 B | FlexRay only (718 490 FR1 records). **No CAN rows in it.** |
| `f11_emu4_ddecan.csv.bin` | 30 020 658 B | raw USB stream, FlexRay format 3 **and** CAN records (type 0x43) |

**Not in the archive:** `f11_emu4_ddecan.csv.console.log`, `.can.csv`, `.savvy.csv`, and **no `f11_emu4_noddecan*` files at all**.
The PT-CAN capture was rebuilt from the .bin with the logger's own replay:
`fr_ts_logger.py --replay f11_emu4_ddecan.csv.bin -o replay/f11_emu4_ddecan.csv --can --no-console-log` →
`replay/f11_emu4_ddecan.csv.can.csv`. Replay summary: frames=718490, hw_ts 99.7 %, seq_gaps 0, **can=110117 (bus2, 128 IDs)**,
bytes_skipped 0. That matches the console-log numbers quoted to me (110117 frames, 128 IDs), so this is the same capture.
The replayed FlexRay CSV is identical in row count to the shipped one.

**Consequence:** the with/without-DDE-CAN diff (task 2b) **cannot be done**: there is no noddecan run in this archive.

## 1. Timeline [E]

| event | t (s) | Europe/Dublin | source |
|---|---|---|---|
| first CAN frame (0x510 NM) | −0.012 | 14:31:15.11 | PT-CAN |
| first FR record (null frames slot 70) / first non-null FR | 0.000 / 0.227 | 14:31:15.12 | FR (host_time of first record) |
| emulator 0x5C wake window | 0.646 – 2.246 (21 seen, 1 in a capture hole) | | FR |
| emulator statics 0x28/0x29/0x3D, 0x26 | 0.574 / 0.584 | | FR |
| real DDE CAN TX starts (0x3F9, then 0x8F/0xA5/0xD9/0xDC/0xF3, 0x2C4/0x281) | 0.427 / 0.447–0.450 / 0.600 | | PT-CAN |
| 0x12F ST_KL 6 → 9 / → **A (KL15 on)** | 5.432 / **5.532** | 14:31:20.65 | PT-CAN 0x12F d2 low nibble |
| first 0x103 / first 0xF1 b3 = 0x20 | 5.477 / 5.577 | | FR |
| DSC ready (0x2F c1 b4 0e→02, b5 →20) | 6.564 (KL15 + 1.032) | | FR |
| 0x12F ST_KL A → 7 / → 6 (**KL15 off**) | **73.972** / 74.072 | 14:32:29.09 | PT-CAN |
| 0xF1 b3 = 0xE0 (last 0xF1) | 73.986 | | FR |
| last 0x103 | 74.076 | | FR |
| last DDE CAN NM 0x512 | 119.901 (KL15 off + 45.93) | | PT-CAN |
| last emulator FR 0x5C | 120.964 (KL15 off + 46.99) | | FR |
| last vehicle FR 0x5C | 124.714 (2.0 s before silence, as runs 3/4/5) | | FR |
| last CAN / last FR frame (bus sleep) | 126.625 / **126.720** | 14:33:21.84 | |

KL15 on for 68.44 s. Bus sleep **52.75 s after KL15 off** (run 3 v1c: 52.4 s). The console-log times quoted to me
(KL15 on ~16 s, off ~84 s) sit ~10.5 s later than these. **Resolved with the console log (§9.1) [E]:** the console `[ N s]` clock
starts at logger start, and the first frame came 10.50 s later (device clock 103.457 → 113.954 s). So console = t + 10.50 s for ddecan
(console off_ago 0.9 s at [85.4 s] gives off at 84.5 = 73.99 + 10.50).

**Capture holes (logger artefact, not bus):** every 5.0 s (t ≈ 1.75, 6.75, …, 121.75; 25 holes) both FlexRay (~90–115 ms,
18–22 cycles) and CAN (~76–89 ms) stop recording. The period equals the logger's `--status-interval 5.0` emu-stats
control transfer [H: the stats read stalls recording]. Counters prove the frames were sent: FB L7 counter jumps by exactly 2,
CAN 0xA5/0xD9/0xF3 counters skip 4–5 values at each hole. Right after some holes there are a few duplicated/reordered FR
records (0x28 counter repeats) and the CAN CSV has 377 out-of-order rows (≤ 60 µs, one 45 ms); scripts sort CAN by time.
2279 FR records (0.3 %) have no hardware timestamp (`rec_flags 0x03`), all at t ≈ x.9 s after the holes.

## 2. FlexRay: emulator health and pass checks [E]

### 2.1 Emulator slots (FR1; L = payload bytes + 1, as in the run 3 report)
| slot | n (non-null) | cycle | period | first / last t | gaps (beyond the 5 s capture holes) | content |
|---|---|---|---|---|---|---|
| 0x26 | 6212 (+18586 null) | c%4=1 | 20 ms | 0.584 / 126.702 | none | E2E 6212/6212 valid |
| 0x28 | 12402 (+12398 null) | c%4=1,3 | 10 ms | 0.574 / 126.712 | none | E2E b1/b9 c1 6211/6211, c3 6191/6191 |
| 0x29 | 6194 (+18609 null) | c%4=3 | 20 ms | 0.574 / 126.712 | none | E2E 6194/6194 |
| 0x3D | 6206 (+18609 null) | c%4=3 | 20 ms | 0.575 / 126.713 | none | E2E 6206/6206; sync/startup bits 0 (EMU_SYNC_BITS=0) |
| 0x5C (c%16=1) | 1442 | c%16=1 | 80 ms | 0.646 / 120.964 | wake window, then off until KL15 | `00 00 00 00 ff ff 00 12` |
| 0x142 L31 | 6207 | c%4=3 | 20 ms | 0.577 / 126.716 | none | phase 0 → 1 at 3.097 → 2 at 7.097 (KL15 + 1.565); b1 counter, only breaks at holes |
| 0x10A L5 | 69 | c%8=2 | 1 s | 5.692 / 73.691 | none | `00 ff ff ff`, first at KL15 + 0.160, **stops at KL15 off** ✔ |
| 0x68 L3 | 26 | c%8=5 | 5 s | 0.586 / 125.584 | none | `c0 ff` |
| 0xE6 L9 | 127 | c%8=2 | 1 s | 0.612 / 126.610 | 1 skip+retry (1.04/0.96 s) | CRC (K 0x04) 127/127, counter from 1, no breaks |
| 0xE8 L9 | 127 | c%8=3 | 1 s | 0.577 / 126.575 | none | `00 00 00 9a 29 f3 56 f0` constant |
| 0xE8 L5 | 14 | c%8=0 | 5 s | 5.722 / 70.721 | none | `86 78 fe fe`, first at KL15 + 0.190, **stops at KL15 off** ✔ (next would have been 75.72) |
| 0xFB L5 | 26 | c%8=5 | 5 s | 0.587 / 125.585 | none | counter ok |
| 0xFB L7 | 606 | c%8=6 | 200 ms | 0.592 / 126.590 | 5 skip+retry pairs (0.24/0.16 s); 25 × 0.4 s = capture holes (counter +2) | run 3 had 29 skip pairs → much better |
| 0xFB L9 | 26 | c%8=3 | 5 s | 0.577 / 125.575 | none | `…ff 7f…` pre-KL15, `…04 00…` from KL15 on, **never back** (§3) |

No emulator slot missing; nothing missing apart from the capture holes.

### 2.2 Vehicle reaction (same checks as run 3)
| check | run 4 (v1d) | result |
|---|---|---|
| 0x2F c1 b4 0e→02, b5 →20 | KL15 + 1.032 s (c3: + 1.022); back to 0e/30 at 74.003 | **PASS** |
| 0x3F c1 b4 fb→f3 | KL15 + 1.033; back to fb at KL15 off | **PASS** |
| 0x2B c3 b11/b12 = e0/f8 | unchanged | **PASS** |
| 0x70 L9 b3/b4 = c8/20 | from 1.44 / 1.34 s | **PASS** |
| 0x119 L7 b3/b5 | 08 → e2 at 2.262 (pre-KL15), as before | same as run 3 |
| 0xB4/0xCF/0xB7, 0x90/0x93/0xAB, 0xBE | 0 frames | **PASS** |
| 0xEA | 164 frames (vehicle RX PDU, as in earlier runs) | n/a |
| cold wake | 0xFC 34.91, 0xFE 34.92, 0x115 36.31 | cold, as run 3 |
| sleep | 52.75 s after KL15 off, no re-wake | **PASS** |
| 0x5C NM release | KL15 off + 46.99 s (target 47 s); real DDE's CAN NM 0x512 released at + 45.93 s | **PASS** |
| KL15-gated PDUs (0xE8 L5, 0x10A) | start on the KL15 edge, stop at KL15 off | **PASS** (fixes run 3 item 5) |

## 3. Stale-ign bug: confirmed, but it is a content bug, not a "kept sending" bug [E]

* Last emulator frames after KL15 off (73.972): **0xE6 126.610 (KL15 off + 52.64 s)**, 0xE8 L9 126.575 (+52.60),
  0xFB L7 126.590 (+52.62), 0xFB L5/L9 125.585/125.575, 0x68 125.584, 0x142 126.716, i.e. until 0.1 s before bus silence.
* **Sending them after KL15 off is what the real DDE does.** In the real-DDE ign-only MITM capture
  (`captures/mitm/F11_mitm_ignonoff.csv`, KL15 off ≈ 15.2–15.3 s from the last 0x10A), the DDE keeps 0xE6, 0xE8 L9,
  0xFB L5/L7/L9, 0x68 and 0x142 until bus sleep (last 0xE6 63.02 s, FB L7 63.31 s, end 63.56 s). Only 0x10A and 0xE8 L5
  stop, and v1d now stops them too.
* **What is stale is the content.** The emulator switched its "ign" content at KL15 on and never switched back:

| PDU | emulator before KL15 | emulator KL15 on (switch at) | emulator after KL15 off | real DDE after KL15 off |
|---|---|---|---|---|
| 0xFB L9 | `1f 00 ff 7f 00 f3 ff ff` | `1f 00 04 00 00 f3 ff ff` (5.577) | **still `04 00`** to 125.575 | MITM: back to `ff 7f` at 15.515 (first L9 after KL15 off) |
| 0x29 c3 g1 (PDU53, CAN twin 0x8F) b8 | `80` | `10` (5.534) | **still `10`** | this run's real 0x8F b7: `10` → `c0` at 74.003 (KL15 off + 0.031) |
| 0x28 c1 g1 (PDU51, twin 0xA5) b3–b5 | `c1 17 7c` (−31 Nm) | `a7 79 9a` (212 Nm) (5.544) | still ign | real 0xA5 also keeps its KL15 torque after off, so no change needed |
| 0xE6 | `80 64 45 44 23 47` (ign content from wake) | same | same | real 0x3F9 (same layout) also unchanged at KL15 off |

  So: the flag that selects pre-ign/ign content latches at the first KL15-on and ignores the KL15-off edge (consistent with
  the "ign flag stale" note from the console log). Fix [H]: drive the content switch from the v1d `kl15.on` state (as the
  E8 L5/0x10A gate already is), revert FB L9 to `ff 7f`, and set an "after off" state for PDU53 b8 = `c0` (0x8F walk 8 → 1 → c).
  Damien saw no dash fault, and the car slept normally, so this is cosmetic in this test.
* Also visible: emulator 0xE6 sends ign-on content from wake (run 3 v1d item 6, pre-ign 0xE6 content, still open); the real
  DDE's 0x3F9 starts with `80 34 3c 30 00 47` and reaches `80 64 … 23 47` at ~1.4 s, before KL15.

## 4. PT-CAN (listen-only on the V2 board, logger "bus 2") [E]

**Which bus:** the DDE's 10 ms PT-CAN2 set (0xA6, 0xA7, 0x145, 0x137) and PT-CAN2-only 0x1BA/0x33D/0x2B1 are **absent**;
0xA5/0xD9/0xDC/0xF3 come at 20 ms, 0x8F at 30 ms, 0x3F9 at 1 s. So this is **PT-CAN** (Ian's node 0 / capture bus 3),
the bus the plan's MUST set is for.

128 IDs, 110 117 frames, full table in `f11_emu_run4_ptcan_ids.csv` (n, DLC, first/last, median/min period, max gap,
counts before KL15 / during / after, number of payloads, modal KL15 payload, attribution). Highlights:

### 4.1 Frames attributable to the DDE (flash TX table, PT-CAN)
| ID | n | period | first t | last t | notes |
|---|---|---|---|---|---|
| 0x0A5 | 6212 | 20 ms | 0.447 | 126.600 | E2E 0x09 |
| 0x0D9 | 6210 | 20 ms | 0.450 | 126.603 | E2E 0x43 |
| 0x0DC | 6210 | 20 ms | 0.450 | 126.604 | E2E 0x5A, DLC 6 |
| 0x0F3 | 6215 | 20 ms | 0.450 | 126.603 | E2E 0xA6 |
| 0x08F | 4142 | 30 ms (min 29.5, steady) | 0.447 | 126.590 | E2E 0x20 |
| 0x3F9 | 137 | 1 s + on change | 0.427 | 126.440 | E2E 0x37 |
| 0x2C4 | 1238 | 100 ms | 0.600 | 126.593 | constant |
| 0x281 | 1236 | 100 ms | 0.600 | 126.593 | voltage |
| 0x1B9 | 127 | 1 s | 0.599 | 126.593 | `00 ff` |
| 0x381 | 18 | 10 s + event | 0.423 | 120.594 | `0c f2` / `7c f0` / `0c f0` / `79 f0` |
| 0x3B3 | 36 | 5 s + event | 0.425 | 125.423 | `1f 00 ff 7f 00 03 f8` (no KL15) / `1f 00 04 00 00 03 f8` (KL15) / `0f 00 00 00 00 00 f8` ×2 |
| 0x512 | 181 | ~640 ms | 0.517 | 119.901 | DDE NM `00 00 00 00 ff ff 00 12`, released KL15 off + 45.9 s |
| 0x351 | 3 | event | 15.509 | 15.829 | `11 05 63 63 fe` ×3 (plan: seen 3×, SKIP) |
| 0x397 | 269 | ~400 ms (1 s + event) | 0.425 | 126.443 | plan: DDE PT-CAN2 frame. **It vanishes without the DDE (§9.3)**, so it is DDE-sourced or a copy that needs the DDE; `c0 e2 00` modal |
| 0x3FB | 223 | 1 s + event | 0.425 | 126.423 | owner conflict (plan loop 4). **Settled by noddecan (§9.3): it vanishes without the DDE, so the DDE sends it** |

All cyclic DDE frames start 0.43–0.60 s after wake and run to bus sleep, i.e. **not KL15-gated** (confirms plan §1.3).

Other notable non-DDE frames: 0x12F (terminal status, 100 ms), 0x173 (= FR 0x2F odd payload, 1244/1244 byte-equal), 0x0EF
(20 ms), CAN NM 0x501/0x510/0x518/0x52A/0x55E/0x560 (0x501/0x52A/0x55E/0x560 stop at KL15 off, 0x510 runs to 124.6 s).
0x5E0: `10 03 00 01/02 00 ff ff ff` bursts at 4.1–6.3 s, then `8c 01 00 00 00 0e 00 ff` / `8c 00 00 6b 00 00 00 ff`
every 10 s from 15.5 to 66.0 s. Sender not attributable from one run.

### 4.2 0x592 (DDE check-control)
`40 22 00 15 ff ff ff ff` every ~5.1 s while KL15 on (5.63 … 71.93) and `40 22 00 14…` at 74.03 (after off): CC 34, the MIL.
The DDE also sends other, undecoded 0x592 frames. They are not emulated.

The ZV never asks the car for an immobiliser exchange, and in testing the car never asked for one, so nothing is needed.

## 5. Phase-1 baseline check: real DDE (KL15 on, engine off) vs the plan [E]

E2E rule verified on every frame: **CRC8 poly 0x1D, init 0, xorout 0, over dataID lo, dataID hi, b1..b(DLC−1) → b0**.
Every variant tried (xorout 0xFF, swapped ID bytes, no ID, init 0xFF) gives 0 matches; the plan's rule gives 100 %.
Alive counter = b1 low nibble, 0..14 (15 never sent), **starts at 1 after wake** on all six E2E frames; the only
counter breaks are the 25 capture holes. "x" below = alive nibble.

| ID | period | CRC ok | plan baseline | observed (KL15 on, engine off) | verdict |
|---|---|---|---|---|---|
| 0x0A5 | 20 ms | 6212/6212 | `CC FA b6 69 9b 00 00 f1` | `CC Fx 1a aa a1 00 00 f1` (T1=T2=0xA1A = 269.5 Nm); from 52.9 s alternating with `19 9a a1` (0xA19 = 269.0 Nm). Before KL15 `b8 87 7b` (0x7B8 = −35.5 Nm) | **mismatch b2–b4 (torque value)**: plan 219.5 Nm, here 269.5 Nm; plan's WAKE row (−29.5 or 0 Nm) vs −35.5 Nm here. b1 hi F, rpm 0, b7 f1 match |
| 0x0D9 | 20 ms | 6210/6210 | `CC 1A 00 10 00 e0 7f c0` | `CC 1x 00 10 00 e0 7f c0`, identical in all phases | match |
| 0x0DC | 20 ms | 6210/6210 | `CC 1A 22 d0 07 fe` | `CC 1x 22 d0 07 fe`; at TX start `80 ff ff f0` (1 frame) → `20 d0 07 f1` (0.49–0.69 s) | match (start-up states extra) |
| 0x0F3 | 20 ms | 6215/6215 | `CC 0A 00 c0 f0 ff ff ff` | `CC 0x 00 c0 f0 ff ff ff`, all phases | match |
| 0x08F | 30 ms | 4142/4142 | `CC 1A 00 7d 7c 00 18 10` | `CC 1x 00 7d 8c 00 18 10`; b7 `80` before KL15, `10` KL15, `c0` after off (from 74.003) | **mismatch b4: 0x8c vs 0x7c** (low nibble c = "off" as plan; high nibble 8 vs 7, meaning unknown). b7 walk matches plan §4 |
| 0x3F9 | 1 s + change | 137/137 | `CC 1A 80 64 48 47 23 47` | `CC 1x 80 64 3d 3d 23 47` | layout match; b4/b5 = 13/13 °C here vs 24/23 °C in plan: temperature value only |
| 0x2C4 | 100 ms | none | `00 00 ff 64 64 64 00 f0` | identical, one payload all run (accumulator 0, engine never ran) | match |
| 0x281 | 100 ms | none | `56 f3` (12.8 V) | `33 f3` modal (0x333 = 12.29 V); pre-KL15 ~0x349 (12.6 V); after off 0x336–0x339 | layout match; voltage value only |

**Emulated FlexRay vs this real DDE's CAN twins** (the plan says build both from one state):
PDU52↔0xD9 and PDU57↔0xDC agree byte for byte. PDU51↔0xA5 differ in torque (emulator 0x9A7 = 212 Nm, real 269.5 Nm).
PDU53↔0x8F differ in b5/b4 (`7c` vs `8c`) and in the after-off b8/b7 (`10` vs `c0`, §3). 0xE6↔0x3F9 differ in the
temperatures (emulator fixed `45 44` = 21/20 °C, real `3d 3d` = 13/13 °C) and in pre-ign content. None of this upset the
car in this run [E: clean dash, no DSC/EPS event frames], but the plan's "CAN = FR" rule is not met yet [H: relevant once
the real DDE is gone and the Pico sends both].

## 6. Brake [E]
* 0x173 d7: `e0` (0.235) → `c1` (0.375, one frame) → `c0` (0.405) → `d0` from 0.905 to the end (bit4 standstill).
  Bits 0–1 are 0 for the whole run apart from the single wake frame `c1`.
* 0x12F d6: `38` → `31` (0.357, one frame) → `30` for the rest of the run.
* FR 0x2F payload[7] (c1/c3): d0, with 1–2 c1 frames at wake only.
* **No brake press in this capture.** The wake-instant bit0 blip is the same as in r10md10m. The BLS/BTS question
  (dde_brake_rx.md loop 1) stays open; it needs the stationary press test.

## 7. Pass / fail summary
| check | result |
|---|---|
| archive contents | ddecan FR csv + bin only (CAN recovered from .bin); console logs and the noddecan run arrived later (§9) |
| emulator slots 0x26/28/29/3D/5C/142/10A/68/E6/E8/FB present, valid E2E | **PASS** (no gaps other than the logger's 5 s holes; 1 E6 and 5 FB L7 skip+retry) |
| KL15 detection (0x12F A at 5.532 / 7 at 73.972; 0xF1 0x20 at 5.577 / 0xE0 at 73.986) | **PASS** |
| KL15-gated 0x10A / 0xE8 L5 | **PASS** |
| 0x5C NM release / sleep | **PASS** (+46.99 s; sleep +52.75 s) |
| DSC/ICM/EPS behaviour | **PASS** |
| pre-ign/ign content follows KL15 off | **FAIL** (FB L9, 0x29 PDU53 b8 stay ign; §3) |
| DDE PT-CAN E2E rule (plan §1.4) | **PASS** 100 % on 0xA5/0xD9/0xDC/0xF3/0x8F/0x3F9 |
| plan baselines | 0xD9, 0xDC, 0xF3, 0x2C4 exact; 0x3F9/0x281 value-only differences; **0xA5 torque and 0x8F b4 differ** |
| with/without DDE-CAN diff | done in §9 (noddecan arrived later) |
| brake presses | none in the capture |

## 8. Next
1. ~~Get the console log and the noddecan run~~ done, see §9.
2. v1d.1: drive the pre-ign/ign content from `kl15.on` (FB L9 back to `ff 7f`, PDU53 b8 `c0` after off).
3. Before the Pico sends the PT-CAN set: take 0xA5 torque (0xA1A) and 0x8F b4 (0x8c) from this run, or understand why they
   differ from the plan's capture; keep the FR twins equal to the CAN values.
4. Logger: the 5 s emu-stats poll costs ~100 ms of capture every 5 s; consider a longer status interval for car runs.

## 9. noddecan run: DDE powered off (12 V off, absent on PT-CAN *and* on its own FlexRay side), emulator v1d still running

Tags: [E] = measured in the capture, [H] = hypothesis.

### 9.1 Files and time base [E]
- Second archive `01M696WQ9B99MTAKZKWQ8826D4.7z` contains `f11_emu4_noddecan.csv.bin` (42.3 MB) and `f11_emu4_noddecan.csv.can.csv` (8.4 MB).
  **The FR `.csv` is not in it**, so it was rebuilt with `--replay f11_emu4_noddecan.csv.bin -o replay/f11_emu4_noddecan.csv --can`.
  - Replay result: 1,034,519 FR1 frames, hw_ts 99.6 %, seq_gaps 0, can = 115,649, 116 IDs, bytes_skipped 0.
  - These numbers equal the console's `done:` line. The rebuilt CAN file matches the shipped `.can.csv` row for row.
- Console logs:
  - `01M696SYW8XAWZ9BJ3P03C7EM6.log` (noddecan, logger started 14:42:29 GMT)
  - `01M6966ZY8YHC8H6CRZ2VXTVEV.log` (ddecan, logger started 14:31:04 GMT)
  - Both are copied to `/workspace/f11-emu-run4/*.console.log`. Times are Europe/Dublin = GMT today.
- **Time offset (resolved).** The console `[ N s]` clock counts from logger start. Below, t = s after the first FR frame.
  - ddecan: console = t + 10.50 s.
  - noddecan: console = t + 6.37 s (device clock 25.815 s at start; first frame at 32.182 s).
  - Check against the console: off_ago 0.8 s at [80.3 s] puts off at 79.5 s console, which is 0xF1 `e0` at t 73.08 + 6.37.
- Logger holes are the same 5 s / ~100 ms artefact as before (here at t ≈ 3.52 + 5k s).

### 9.2 FlexRay checks (noddecan) [E]

| item | noddecan | ddecan (for comparison) |
|---|---|---|
| KL15 on | 0x12F ST_KL 9 at 5.936, A at 6.006; F1 `0x20` at 6.067; first 0x103 at 5.997 | A 5.532 / F1 5.577 |
| KL15 off | 0x12F ST_KL 9 at 73.067, **8 at 73.442 and stays 8**; F1 `0xE0` at 73.076 | 7 at 73.972 → 6 at 74.042; F1 73.986 |
| KL15 duration | 67.06 s | 68.44 s |
| DSC ready (0x2F c1 b4 0e→02) | 7.044 = ON + 1.038 (reverts at 73.083); 0x3F b4 fb→f3 at 7.045 | ON + 1.032 |
| DSC/EPS event IDs 0xB4/CF/B7/90/93/AB/BE/88/67 | none (0xEA 161 frames, vehicle) | none (0xEA 164) |
| 0x70 L9 b3/b4 | `c8 20` | same |
| cold-wake markers | FC 0.08, FE 33.04, 0x115 35.91 | FC/FE 34.9, 0x115 36.3 |
| emulator statics 0x26/28/29/3D | all cycles present, E2E 100 % valid in all 9 groups, no gaps other than logger holes | same |
| emulator dyn E6/E8L9/FB5/7/9/0x68/0x142 | all present at nominal period until log end (183.3 s); E6 CRC 184/184 | same until sleep |
| KL15-gated 0x10A / E8 L5 | 6.21–72.21 / 6.24–71.24, i.e. correctly gated | same |
| emulator 0x5C (c%16=1, `..0012`) | wake window 0.166–1.766 (21 frames); KL15 run 6.006 → **120.004 = off + 46.94 s**; console nm=quiet, 1447 sent | +46.99 s |
| bus sleep | **none**; FR and CAN still active at 183.3 s = **off + 110.25 s** when logging stopped | sleep at off + 52.75 s |
| stale-ign content | same bug: console shows `ign=on` all the way to the stop. FB L9 stays `..04 00 00 f3 ff ff`, 0x29 c3 b8 stays `0x10` after off | same |

The emulator behaved identically in both runs. It is not what keeps the bus awake: its 0x5C stopped on schedule at +46.94 s, and in ddecan the same firmware let the bus sleep.

### 9.3 PT-CAN diff: ddecan (128 IDs) vs noddecan (116 IDs) [E]
Full per-ID table: `f11_emu_run4_ptcan_diff.csv`. It has 132 rows. Columns per run: count, period, first, last relative to KL15 off, KL15-phase and after-off counts, and distinct payloads. A byte-level diff note and an interpretation are added per ID.

- **Vanished (16) = DDE TX set:** 0x8F, 0xA5, 0xD9, 0xDC, 0xF3, 0x1B9, 0x281, 0x2C4, 0x351, 0x381, 0x397, 0x3B3, 0x3F9, 0x3FB, 0x512, 0x592.
  - This settles two open questions. 0x3FB is DDE-sent (the plan's loop-4 owner conflict). 0x397 is not an independent gateway copy: it disappears with the DDE.
  - 0x592 also belongs to the DDE.
- **New (4): UDS traffic.** 0x65E, 0x618 and 0x62A carry ISO-TP positive responses `62 17 04 …` (ReadDataByIdentifier 0x1704).
  - Responders: ECU 0x5E at 4.0 s, 0x18 at 8.0 s, 0x2A at 12.1 s and 62.2 s.
  - Requester: 0xF0. 0x6F0 is only its flow-control frames; the requests themselves are not on this bus.
  - These are plausibly fault/DTC-environment reads triggered by the missing DDE [H; the meaning of DID 0x1704 is not known here].
- **Other ECUs' reactions (content/rate changes clearly tied to the DDE):**
  - **0x12F (terminal status):**
    - d5 bit 2 is set at wake in both runs. In noddecan it is **cleared at 2.96 s, before KL15, and never returns**, which looks like a ~3 s timeout. In ddecan it dipped once at 0.527 and came back at 0.602, 2 ms after the DDE's first 0x1B9/0x281/0x2C4.
    - Off sequence, ddecan: d5 0x85 → 0x45 → 0x05, ST_KL A → 7 → 6.
    - Off sequence, noddecan: d5 0x81 → 0x01, **no 0x45 step**, ST_KL A → 9 → **8, held**, with d4 low bits = 4. In ddecan that d4 value appears only during the KL15-on transition.
    - The d3 high-nibble countdown (6→1, 1 s steps) runs in both runs.
  - **0x3A0 d2:** `CF` at wake in both. ddecan switches to `C0` at 0.56 s, right after the DDE frames start. noddecan stays `CF` for the whole run.
  - **0x2DC d2:** ddecan 01 → 03 at 0.78 s. noddecan stays 01.
  - **0x39A d2:** noddecan 00 → 40 → 44 at 7.37 / 7.47 s (ON + 1.4 s), held to the end. ddecan stays 00.
  - **0x5C0 (CAS check-control):** pairs of `40 12 00 29` / `40 0e 00 29` arrive every 10 s during KL15. ddecan has a single `40 03 03 29/28`.
  - **0x5B8:** `40 f5 00 19` every 6.0 s from 16.4 s to the end. ddecan has 2 frames at wake only (`..18`).
  - **0x34C d0/d1:** ddecan `ff 7f` (KL15) / `7d 00`; noddecan `79 00`. Unexplained.
  - No new 0x5xx check-control-type ID appears.
- **Ambient / unrelated differences** (temperatures, analog values, date/time, counters): 0x163, 0x199, 0x2A5, 0x2F8, 0x301, 0x302, 0x328, 0x32E, 0x330, 0x349, 0x393, 0x5E0 d5 (0e vs 0d). Listed in the CSV and not treated as reactions.
- **Frames that are KL15-gated in ddecan (stop at off ±0.3 s) but keep running after off in noddecan:** 0x1B3, 0xD7, 0x2A5, 0x32E, 0x26A, 0x226, 0x28A/0x28B, 0x2C3, and NM 0x501 and 0x560. On FlexRay, 0x103 and 0x10C keep running too. ddecan stopped both at 74.08 / 74.07.

### 9.4 Why the bus stayed awake [E, cause H]
No NM byte changes value in any of these frames (0x5xx: bytes 0–3 = 0, b7 = node). A node "requests" by transmitting, so "who is still transmitting" is the measure.

| NM frame | ddecan last (rel. KL15 off) | noddecan last (rel. off; log ends +110.25) |
|---|---|---|
| CAN 0x501 (node 01) | −0.04 | **+110.16** (still running) |
| CAN 0x560 (node 60) | −0.34 | **+109.87** (still running) |
| CAN 0x52A / 0x55E | −0.19 / −0.11 | −0.10 / −0.02 |
| CAN 0x518 | +29.91 | +29.80 |
| CAN 0x512 (DDE) | +45.93 | absent |
| FR 0x5C c1 `..0012` (emulator) | +46.99 | +46.94 |
| CAN 0x510 (gateway, node 10) | +50.63 | **+110.18** |
| FR 0x5C c7/c15 `..0010` (gateway, FR side) | +50.74 | **+110.25** |
| FR 0x5C c4 `4c2e0002` / c12 `..29` / c14 `..1c` | +0.05 / +1.69 / +5.14 | +0.39 / +1.95 / +5.16 |

- **What keeps it awake [E]:**
  - In ddecan, the last requesters were the DDE's 0x512 (+45.9) and our FR 0x5C (+47.0). The gateway (node 0x10, on CAN 0x510 and FR 0x5C c7/c15) released ~3.6 s later, and the bus slept at +52.75.
  - In noddecan, **PT-CAN nodes 0x01 (0x501) and 0x60 (0x560) never release.** In ddecan both stopped at KL15 off.
  - So the gateway keeps both networks up, and FlexRay with them.
  - Every other NM participant releases at the same time in both runs.
- **The tested hypothesis "a gateway/DME node waits for DDE 0x512": not supported [E].**
  - In the normal run, nodes 01 and 60 drop at KL15 off, ~46 s *before* 0x512 releases. So they are not waiting for 0x512.
  - The gateway simply follows the last requester.
- **Most likely cause (confidence: medium-high that it is the missing DDE PT-CAN presence; medium on the exact mechanism) [H]:**
  1. The 0x12F sender (the terminal / vehicle-state master) notices the missing DDE ~3 s after wake (d5 bit 2 cleared).
  2. At KL15 off it takes a different path: 9 → 8 instead of 7 → 6, without the 0x45 acknowledge step, and it holds state 8.
  3. Nodes that key their shutdown on that terminal state keep running in their KL15 mode. That covers 0x501/0x560 NM, 0x1B3/0xD7/0x2A5/0x32E/0x26A/… on CAN, and 0x103/0x10C on FR.
  4. The gateway therefore keeps the network up.
  - This is supported by: the d5 bit timing, the different off path, and the set of KL15-gated frames that keep running.
  - Not proven: which DDE frame the state master is missing.
  - Not ruled out: nodes 01/60 react to the missing DDE frames directly rather than via 0x12F.
- **Not the emulator [E]:** the FR side, including the emulator's 0x5C release, was the same in both runs.

### 9.5 What v1e / PT-CAN TX needs [H]
1. **Send the DDE PT-CAN cyclic set from the Pico from wake:**
   - 0x8F at 30 ms.
   - 0xA5, 0xD9, 0xDC, 0xF3 at 20 ms.
   - 0x1B9, 0x281, 0x2C4 at 100 ms.
   - 0x3F9, 0x3FB, 0x397, 0x381, 0x3B3, using the ddecan rates/content and the plan's CRC rule.
   - **0x512 NM** `00 00 00 00 ff ff 00 12` every 640 ms. Release it at the same point as our FR 0x5C (KL15 off + ~46 s).
   - Start within ~0.5 s of wake. The DDE's frames appeared at 0.42–0.60 s, and the d5 bit times out at ~3 s.
   - Use ddecan values for 0xA5/0x8F (§5) and the after-off content the real DDE uses.
2. **Pass criteria for the next noddecan-style run:**
   - 0x12F d5 bit 2 stays set and the off path goes 7 → 6.
   - 0x3A0 d2 goes to `C0` and 0x2DC d2 to `03`; 0x39A d2 stays `00`.
   - No 0x6xx UDS reads.
   - 0x501/0x560 stop at KL15 off.
   - Bus sleep ≈ off + 52 s.
3. **Bisection if (1) is not enough.** Run the CAN set without 0x512, then 0x512 alone, to find the frame the state master watches.
   - If the state still ends at 8 with the full cyclic set, the missing piece is something else the real DDE does.
   - In that case the real DDE has to stay on PT-CAN, or the shutdown has to be forced some other way. That needs a decision from Damien, not an emulator change.
4. Unchanged from §8: fix the stale-ign content (v1d.1). The console confirms `ign=on` after off in both runs.

### 9.6 Brake (noddecan) [E]
- 0x173 d7: e0 (0.085) → c0 (0.155) → d0 (0.785), then d0 for 1792 of 1801 frames to the end. **No brake presses.**
- 0x12F d6 stays `30` all run.
- 0x173 is byte-equal to an 8-byte window of FR 0x2F in 1801/1801 frames.
- Since the brake was not pressed, KL15 on was not an engine-start attempt.

### 9.7 Scripts
In `emu_run4_scripts/`:
- `load4.py`: now parametrised as `fr(run)` / `can(run)` / `both(run)` for the two runs.
- `nod_fr.py`: FlexRay checks, `python3 nod_fr.py noddecan|ddecan`.
- `nm_cmp.py`: NM after-off comparison.
- `diff_ids.py`: per-ID diff. Writes the raw diff; the interpretation column was added by hand into `f11_emu_run4_ptcan_diff.csv`.
