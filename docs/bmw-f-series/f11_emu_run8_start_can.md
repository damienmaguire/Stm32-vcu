# F11 emulator run 8 (v1g.2 fake start, DDE + EGS removed, CAS/BDC kept): PT-CAN start sequence

Analysis date 2026-11-01 (Europe/Dublin). Owned hardware, documentation only.
Labels: **[CONFIRMED]** = directly in the run-8 log. **[ANALYSIS]** = derived. **[HYPOTHESIS]** = not tested.
This note contains IDs, bit positions and timing only.

Companion timeline: `f11_emu_run8_start_can.csv`. Scripts: `emu_run8_scripts/` (`load.py`, `tx.py`, `gear.py`, `mkcsv.py`).

## 0. Inputs and bus labelling

| Item | Finding | Label |
|---|---|---|
| `f11_emu8_v1g.csv.bin` | Raw **fr_ts_logger binary record stream** (format 3, FlexRay plus CAN records). It is **not** a CSV. | [CONFIRMED] |
| `input/f11_emu8_v1g.7z` | Holds only `f11_emu8_v1g.csv.bin`, which is byte-identical to the top-level .bin (`cmp`). | [CONFIRMED] |
| `replay/` | **Already exists.** It was decoded on 2026-11-01 at 10:51 with `fr_ts_logger.py --replay … --can --savvy-csv`. Files: `f11_emu8_v1g.csv` (FlexRay), `.csv.can.csv` (CAN, used here), `.csv.savvy.csv`. | [CONFIRMED] |
| `run8.7z`, `run8_console_attach.log` | **Not present** on the box. The console log used is `f11_emu8_v1g.csv.console.log` (identical to `input/f11_emu8_v1g.console.log`). | [CONFIRMED] |
| Logger "bus 2" | This is **PT-CAN**. It carries vehicle-side 0x12F, 0x173, 0x197 (GWS) and 0x1B3, and the PT-CAN2-only EGS/DDE IDs (0x1AF, 0x0A0, 0x0B0, 0x0F8, 0x145, 0xA6, 0xA7) are absent. It is the only CAN bus in the capture: 87,528 Rx and 40,452 own Tx frames, 111 IDs. | [CONFIRMED] |
| Capture loss | The device reported usb_fifo_dropped = 90,960 records, with FlexRay seq gaps. **CAN is not affected:** 0x12F, 0x173, 0x197, 0xA5 and 0xEF have no gap > 1.07× their cycle across the whole capture. 0x3FD and 0x1B3 each have one 165 ms gap (at +9.3 s and +12.0 s, outside the start window). | [CONFIRMED] |

Time bases: **button** = 0x12F d5 bit7 set (capture t 51.0596 s). **brake** = 0x12F d6 0x30→0x33 (50.2593 s). **KL50** = first 0x12F with ST_KL D (51.4499 s). All times below are in ms.

## 1. CAS terminal states on 0x12F (d2 low nibble = ST_KL)

| rel button | rel brake | rel KL50 | ST_KL | d1 hi | Event | Label |
|---|---|---|---|---|---|---|
| −800 | 0 | −1191 | 6 | 2 | Brake on (d6 → 0x33). 0x173 d7 b0–1 → 3 follows 23.6 ms later. | [CONFIRMED] |
| 0 | +800 | −390 | 6 | 2 | START edge (d5 bit7). It clears at +120. | [CONFIRMED] |
| +170 | +971 | −220 | **9** | 2 | KL_R-like | [CONFIRMED] |
| +235 | +1035 | −155 | **A** | 5 | **KL15** (≥ 0xA) | [CONFIRMED] |
| +290 | +1091 | −100 | **C** | 9 | Start requested. **0x12F cycle drops to 20 ms.** | [CONFIRMED] |
| **+390** | **+1191** | **0** | **D** | B | **KL50 / crank** (≥ 0xD) | [CONFIRMED] |
| +1370 | +2171 | **+979.8** | **C** | 7 | KL50 released. d7 01→00 for this one frame. | [CONFIRMED] |
| +1550 | +2350 | +1159.9 | **A** | 7 | Run. 0x12F returns to the 100 ms cycle. | [CONFIRMED] |
| +1880 | +2680 | +1489.8 | A | 7 | Brake released (d6 → 0x30) | [CONFIRMED] |

- 0x12F d5 bit2 (field42, P secured) = 1 for the whole start. [CONFIRMED]
- The stop walk matches the real car's engine-ran path: A→9→8 (+73.9 s rel KL50), ST_KL 8 held, second press → 7 → 6. d1 hi went 4→2 30 ms after the emulator sent 0x80. Details are in the CSV. [CONFIRMED]

## 2. Starter / terminal-50 request

- **Signal:** 0x12F d2 bits 16–19 (ST_KL) = 0xD; d1 hi nibble = B while cranking. No other PT-CAN frame carried a crank flag. [CONFIRMED]
- **Asserted** at button +390 ms (brake +1191 ms). **De-asserted** (D→C) at +979.8 ms rel KL50.
- **KL50 strictly ≥ 0xD lasted 979.8 ms**, which fits Sam's "~1 s" starter crank [CONFIRMED bus; starter-relay mapping ANALYSIS]. Real-car references: 1042 ms (3krev) and 822 ms (r10md10m).
- Counting the DDE hysteresis that holds KL50 at C, the window is C→A at **1159.9 ms**. From the first C it is 1260 ms. [CONFIRMED]
- **0x1B3 d4 bit4** (the pinned vehicle-side KL50 flag) **was never set** in run 8. 0x1B3 appears only from ST_KL 9 (−205 ms) onward. [CONFIRMED] See the surprises section.

## 3. What CAS needed from the emulator to drop the starter and go to run

Emulator TX around the crank (rel KL50; all on PT-CAN, dir = Tx):

| t | ID | Field | Value | Pinned real DDE |
|---|---|---|---|---|
| −5754 → end | 0x3F9 | bits 28–29 (P secured) | 2 | 2 |
| −101 | 0x3FD | B2 bits 5–7 | 0→1 (P) | P with KL15 |
| −114 | 0x8F | B7 hi | 8→1 (KL15 seen) | −141 |
| +76 / +79 | 0x3F9 / 0xD9 | B1 hi crank nibble | 1→3 | +51…59 |
| +106 | 0x3F9 | B2; B6 b52–53 start-ready | 0x80→**0x81**; 2→1 | +100…111 |
| +116 | 0xA5 | rpm (bit 40, ×0.25) | 0→131 | +94 |
| +126 | 0x8F | B4 lo run nibble | C→2 | +102 |
| +179 | 0xD9 | crank nibble | 3→1 | +152…159 |
| +636…+976 | 0xA5 | rpm ramp | 200→1027 (peak 1059 at +1056) | 0.75–1.0 s |
| **+946.0** | 0x3F9 | B2 / start phase | **0x82** / 2 (start-ready still 1) | +1002 |
| **+971.0** | 0x3F9 | B6 b52–53 start-ready | **1→0** | +1031 |
| **+979.8** | *(0x12F Rx)* | ST_KL | **D→C** | +1042 |

- **[CONFIRMED]** CAS released KL50 on the 20 ms 0x12F slot **8.8 ms after the start-ready = 0 event frame** and 33.8 ms after the 0x82 frame. The 0x12F frame at +960.0, 14 ms after 0x82, was still D.
- **[ANALYSIS]** The trigger is the 0x3F9 run state (0x82 plus start-ready 0), together with rpm ≈ 1000 on 0xA5. The same lag held on the real car: D→C came 11 ms (3krev) and 5 ms (r10md10m) after start-ready 0, and 40 / 27–39 ms after 0x82.
- **[HYPOTHESIS]** Start-ready = 0 (0x3F9 B6 bits 52–53) is the actual release key. Two things keep this from being proven:
  - A 14 ms CAS processing latency for 0x82 cannot be excluded.
  - A CAS rpm threshold on 0xA5/0xF3 cannot be excluded either.

  A bisection run would settle it: send 0x82 with start-ready held at 1, or rpm without 0x82.
- **Static preconditions that held throughout** [CONFIRMED present; their necessity is HYPOTHESIS]:
  - 0x3F9 bits 28–29 = 2, giving 0x12F field42 = 1.
  - 0x3F9 start-ready = 2 before the crank.
  - 0x3FD = P.
  - 0x39A B4 b0–1 = 2 (P-lock), d2 = 00.
  - 0x197 GWS idle.
  - Brake mirror 0x12F d6 = 0x33.

## 4. Immobiliser

The ZV never asks the car for an immobiliser exchange, and in testing the car never asked for one, so nothing is needed.

- 0x592 carries only the emulator's own CC 34 (MIL off) frames, sent at −56.6 ms and then every ~5.1 s.
- 0x5C0 carries the CAS check-control frames: CC 148 at −3.8 s, then CC 18 at +988 and CC 14 at +1488, repeating every 10 s, and both inactive at stop.

## 5. Unanswered gear / EGS requests

- **0x197 (GWS):**
  - B1 hi = 0, B2 hi = 0 and B3 = 0xC0 for the whole capture. There was no lever move and no P-button press, so no gear request was made.
  - The 30 ms cycle had no gaps.

  [CONFIRMED]
- **0x3FD / 0x39A / 0x396:** emulator-supplied and steady (P, P-lock 2, d2 00 — no 0x44 fault flags). [CONFIRMED]
- **UDS diagnostics, one-shot and not a storm** [CONFIRMED; the sender of 0x6F0 is not pinned]:
  - Node 0xF0 sent requests on 0x6F0:
    - a functional request at +11.1 s, answered by 0x01, 0x2A and 0x5E (0x601, 0x62A, 0x65E);
    - a physical request to **0x18 (EGS) at +20.39 s**, with **no 0x618 reply**;
    - a physical request to **0x17 at +20.91 s**, with **no 0x617 reply**.
  - **Neither was retried.**
  - **[HYPOTHESIS]** These are a KOMBI/head-unit or gateway poll that will log "no response" for the missing nodes. They are not part of the start sequence.

## Surprises vs the pinned `dde_start_sequence.md`

1. **0x12F runs at a 20 ms cycle while ST_KL ∈ {C, D}**, from −100 to +1060 ms, then returns to 100 ms. [CONFIRMED run 8] The legacy 3krev log shows the same: D frames are 19–21 ms apart [CONFIRMED]. The pinned note's "100 ms + event" description, and its claim that D→C at +1042 is "off the 100 ms grid", should be corrected: D→C is the next 20 ms slot.
2. **0x1B3 d4 bit4 (pinned vehicle-side KL50 flag) never set** in run 8. [CONFIRMED] Either it is DDE-sourced or DDE-dependent, or the bit position in the pinned note needs rechecking. [HYPOTHESIS]
3. **C→A took 180 ms** after D→C. The real car took 100 ms. [CONFIRMED] The cause is unknown. [HYPOTHESIS: the CAS waits on a further run confirmation.]
4. The emulator sends **0x592 CC 34 OFF only**, with no bulb-check ON at KL15. [CONFIRMED] This is cosmetic per the pinned §4.7.
5. Emulator timing is slower than the real DDE:
   - crank nibble +76 ms vs +51…59 ms;
   - first rpm +116 ms vs +94 ms;
   - **0x92 at 26.3 ms after ST_KL 8**, against the pinned ≤ 20 ms target. [CONFIRMED]

   None of these blocked the sequence.
6. Brake to KL50 took 1191 ms (real car: 922 ms). This is driver timing only, because button to KL50 was 390 ms against 381–400 ms on the real car. [ANALYSIS]

## NEEDS CAPTURE
- FEM/CAS fault memory read after run 8 (missing EGS/0x17 diag replies).
- Bisection of the KL50 release key: 0x82 with start-ready 1, start-ready 0 without 0x82, and rpm only.
- The 0x1B3 KL50 bit on a real-DDE start, to recheck the pinned bit position.
