# F11 emulator run 7: firmware v1f (DDE removed, EGS fuse pulled, Pico emulating FlexRay + PT-CAN)

**Inputs:** `/workspace/f11-emu-run7/f11_emu7_v1f.csv.bin` and the matching `.console.log`
**Replay:** `fr_ts_logger.py --replay … --can --savvy-csv` (v1f dist), written to `/workspace/f11-emu-run7/replay/`
**Checker:** `check_emu_capture.py`, output saved in `/workspace/f11-emu-run7/check_emu_capture_output.txt`
**Scripts:** `/workspace/f11-emu-run7/scripts/` (`load7.py`, `seg.py`, `brake_hunt.py`, `brake7.py`, `brake7b.py`, `ids7.py`)
A colleague worked on the brake in parallel. Their scripts are in `analysis/emu_run7_brake_scripts/`, and they added a run 7 section to `dde_brake_rx.md`.

**How times are given:** *t* is seconds from the first CAN frame in the capture. Add **7.93 s** to get the time shown in the console log.

---

## 0. Summary

| Question | Answer |
|---|---|
| Did the car accept the emulator? | **Yes.** KL15 on/off is exactly like run 6. No new engine or gearbox warnings came from other control units. The bus went to sleep on time (OFF + 52.6 s). |
| Is the 20 ms timing fixed? | **Yes, by the device's own counters.** 0x0A5/0x0D9/0x0DC/0x0F3 now go out 250–256 times per 5 s window, i.e. 50.0 per second. v1e managed 244 (2.5 % short). The send queue was never full: no failures and zero error counters. |
| Were our own sent frames logged (dir=Tx)? | **No. The capture contains zero Tx rows.** The console banner shows the run used the **v1e logger** (the one without the "own TX frames logged" text). The old logger never asks the device for Tx records. This is an operator issue, not a firmware fault, but it means the frame-by-frame period checks (≥ 99.5 % of slots, 0x397 content, 0x518/0x512 stop times) **could not be checked from the bus**. Where possible they were checked from the console counters instead. |
| USB drops (`usb_fifo_dropped = 187 933`, about 13 %) | **New in v1f and caused by the firmware, not by Tx logging** (no Tx records were sent). See §3. |
| Brake | Three clear presses. Switch signals, a **brake pressure** signal and a **pedal-travel-like** signal were all found, with numbers. See §4. |
| **Verdict** | **v1f: PASS on car behaviour and on the 20 ms fix. Not a full pass on the v1f acceptance list**, because own-TX logging went untested (wrong logger) and capture quality has regressed (13 % of frames lost on USB). Repeat with the v1f/v1g logger, and fix the USB drops in v1g. |

---

## 1. What happened in the capture (three wake-ups)

| Wake | Time (t) | Console time | What happened |
|---|---|---|---|
| 1 | 0 – 76.5 s | 7.9 – 84.4 s | Car woken, no ignition. START pressed many times between 4.8 and 23.5 s (key not recognised: "dead key"). |
| 2 | 115.4 – 257.4 s | 123.3 – 265.3 s | **Ignition (KL15) ON at 119.66 s**, three brake presses, **OFF at 204.71 s** |
| 3 | 270.6 – 306.0 s | 278.6 – 313.9 s | Short wake, no ignition |

- **Sleep after ignition off:** last CAN frame at OFF + 52.63 s, last FlexRay frame at OFF + 52.72 s.
  - Run 6: +52.67 / +52.77. The real-DDE baseline is the same. **PASS.**
- **FlexRay sync** was lost 3 times, once at each bus sleep. That is normal.

## 2. Our emulated frames (from console counters, since no Tx rows exist)

### Rates

The console prints a counter for each ID every ~5 s.

| ID | Target | Increase per 5 s window | Result |
|---|---|---|---|
| 0x0A5 / 0x0D9 / 0x0DC / 0x0F3 | 20 ms | 250–256 (v1e: 244) | **fixed** |
| 0x08F | 30 ms | 167–170 | OK |
| 0x2C4 / 0x281 / 0x3FD | 100 ms | 50–51 | OK |
| 0x397 | ~250–600 ms, with blips | 5–20 per window. Mean spacing: **479 ms** over wake 2, **490 ms** while KL15 on, 457 ms in wake 1, 401 ms in wake 3 | Inside the 250–600 ms target. Content (blips, dd/df walk after OFF) not visible. |
| 0x3FB | 1 s + start-up burst | burst of 30–69, then 5 per window | OK |
| 0x592 | MIL frame every ~5 s during KL15, plus one at OFF | **18 total** = 17 during KL15 + 1 at OFF | Matches design. Content not visible. |

### Totals over the whole run

| Measure | Value |
|---|---|
| Frames sent | 69 124 |
| "fifo-full" | 0 |
| "fail" | 0 |
| TEC / REC (error counters) | 0 / 0 |
| Faults | none |

### Whole-wake check of the 20 ms rate

The expected count is "wake length ÷ 20 ms".

| Wake | Sent / expected | Shortfall |
|---|---|---|
| 1 | 3799 / 3824 | |
| 2 | 7070 / 7096 | |
| 3 | 1739 / 1765 | |

In every wake the shortfall is **about 25 frames, i.e. 0.5 s**. That is the start-up delay after the bus wakes. **Once running, the rate is a steady 50.0 per second. No frames are lost.**

### Stop times after OFF (console resolution ~5 s)

| ID | Last increase | Target |
|---|---|---|
| 0x518 | OFF + 29.8 … 34.9 s (one frame after the 29.8 s print) | ~OFF + 29.8 s |
| 0x512 | OFF + 45 … 50 s | ~OFF + 45.8 s |

**Consistent with the targets.**

Without KL15 (wakes 1 and 3), 0x518 stops about 30 s after wake and 0x512 after 4 frames. This is as designed.

### Other bus checks

- **No foreign copies** of our IDs: no other node sent any of the 19 emulated IDs. **PASS.**
- **ST_KL (0x12F):** wake 1 went 4 → 5 → 6. Wake 2 went 6 → 9 (119.56) → **A (119.66)** … **7 (204.71) → 6 (204.79)**. **PASS** (A → 7 → 6, same as run 6).
- **MIL (0x592):** our frames are not visible, see above.
  - **No other control unit raised an engine check-control message.**
  - The CAS check-control frames at KL15 ON are the same as in run 6 (CC 771 set/clear).
  - Whether the dashboard MIL was off has to come from Damien's eyes.

### New IDs and messages vs run 6 (`can_id_diff_run6_vs_run7.csv`, `check_control_run6_vs_run7.csv`)

| Type | New vs run 6 | Notes |
|---|---|---|
| PT-CAN IDs | **0x62A and 0x6F0** (3 frames, at OFF + 5.0 s) | A routine diagnostic read, "ReadDataByIdentifier 0x1704", answered by node 0x2A. The same exchange appeared in the real-DDE run 4. **Harmless.** |
| FlexRay IDs | **0x101** | Only 4 frames in the whole capture (17.95, 17.99, 116.98 and 272.50 s), each with a 2-byte payload of `00 00`. Rare dynamic-segment frames near wake-up, not linked to the brake or the emulation. Harmless. |
| Check-control (CC) from 0x5C0 (CAS/FEM) | **CC 66** (set 6.35 s, cleared 12.25 s; set again 27.55 s, cleared 37.55 s) and **CC 652** (set 12.15 s, cleared 22.15 s) | Only in **wake 1**, while START was being pressed with an unrecognised key. Almost certainly key/start related, not the engine emulation. Not present during the KL15 wake. |
| Other CC frames | Same set as run 6 | Counts are higher only because run 7 has three wakes. |
| Lost IDs | None | |

---

## 3. USB drops (`usb_fifo_dropped = 187 933`)

### What it is

The Pico keeps a 256-frame queue towards the PC. A frame is dropped when that queue is full. 187 933 frames (≈13 %) were lost **on the way to the PC only**. The car bus itself was fine, and our transmit was fine (0 errors).

### Pattern

- 39 489 separate gaps of 1–54 frames each (median 4), about 600–1 300 lost frames per second (`usb_drop_per_second_run7.csv`).
- Drops are continuous for the whole time the bus is awake. They start 2.7–7.4 s after each wake-up and stop at sleep.
- The gaps sit in the busy part of each 5 ms FlexRay cycle. This means the PC side is simply **not emptied fast enough**: the queue sits nearly full and spills over on every traffic peak.

### This is new and not a USB bandwidth limit

Run 6 (v1e firmware, same logger, same car, ~5 800 FlexRay frames/s) had **0 drops**.

### It is not caused by logging our own Tx frames

No Tx records were produced: the old logger never switched them on, and the capture has 0 Tx rows. So the Tx records themselves cannot have filled the queue.

### Most likely cause: firmware v1f main-loop load

- In v1f, every pass of the main loop reads the CAN chip's "transmit event" queue over SPI (`drain_tef()` in `ptcan_emu_poll`).
  - This happens **even when the PC did not ask for Tx records**. The source comment says "Always drained while TX is on".
  - Each read is a blocking 4 MHz SPI transaction of roughly 20–25 µs. Each sent frame adds another read plus an acknowledge write.
- The same loop also services USB. Thousands of these reads per second take a significant share of the time that used to go to emptying the USB queue.
- v1f also added a 1 ms console-drain timer, which adds interrupts on the same core.
- Run 6 → run 7 differs only in firmware (v1e → v1f), so the regression is in one of these v1f changes. `drain_tef()` per pass is the main suspect.
- **This could not be proven from the capture alone.** A bench A/B test (TX on vs off, or TEF reading disabled) would settle it.
- **v1g (current HEAD 58cf388) still calls `drain_tef()` on every pass**, so v1g will most likely drop frames too.

### Suggested fix for v1g logging

1. Only read the transmit-event queue when frames are actually waiting to be confirmed (sent − confirmed > 0). Better still, use the chip's interrupt pin/flag (TEFIF), or limit the read to once per 1 ms.
2. Skip the per-frame read entirely when Tx records are off and only counters are needed. Counting can come from the TX FIFO sequence numbers.
3. Raise the SPI clock from 4 MHz towards 10–17 MHz (within the MCP251863 spec) so each read is 3–4× shorter.
4. Optionally make the USB queue bigger than 256 frames, so short stalls do not spill.
5. Make `check_emu_capture.py` **FAIL when `usb_fifo_dropped` > 0**, and when the console banner shows an old logger.
6. Always run the logger that ships with the firmware (v1f/v1g `fr_ts_logger.py`). Its banner must say "own TX frames logged as dir=Tx".

---

## 4. Brake: three presses with ignition on (gentle, firmer, maximum)

The car was stationary throughout: the standstill bit stays set.

### 4.1 Brake switch signals (on/off)

| Signal | Where | Pressed value | Released value |
|---|---|---|---|
| PT-CAN **0x173 d7** (0-based byte 7) | copy of FlexRay 0x2F | bits 0+1 = switches, bit 2 = "pressure above threshold" → `D3` (switch only), `D7` (switch + pressure) | `D0` |
| PT-CAN **0x12F d6** (0-based byte 6) | copy of FlexRay 0x74 byte 7 (1-based) | `0x33` (bits 0+1) | `0x30` |
| FlexRay **0x74** byte 7 (1-based), cycles 2 mod 4 | | bits 0+1 = 3 | 0 |
| FlexRay **0x2F** odd cycles byte 8 (1-based) | | same as 0x173 d7 | |

**Edge times (t, seconds; add 7.93 for console time):**

| Press | 0x12F / FR 0x74 switch ON | 0x173 / FR 0x2F switch ON | Pressure bit 2 ON | Pressure bit 2 OFF | 0x173 switch OFF | 0x12F switch OFF |
|---|---|---|---|---|---|---|
| 1 gentle | 141.538 | 141.559 | 142.018 | 144.638 | 144.838 | 144.818 |
| 2 firmer | 146.923 | 146.938 | 147.138 | 151.318 | 151.528 | 151.508 |
| 3 maximum | 155.563 | 155.578 | 155.848 | 162.778 | 163.178 | 163.163 |

- The two switch bits (0 and 1) **always changed together**, even at 10 ms resolution. Which bit is the brake-light switch and which is the second switch is **still unresolved**. A very slow, light press would be needed to split them.
- The body-side copy (0x12F / FR 0x74) changes **15–20 ms before** the DSC copy (0x173 / FR 0x2F).
- The switch closes **before any pressure builds**: 55–115 ms before pressure starts (see 4.2). It opens about when pressure returns to zero.
- Pressure bit 2 switches on when the pressure signal below reaches **about 5–6 bar** (≈ −450 Nm torque) and off at about 4–5 bar.

> **Checker bug:** `check_emu_capture.py` reads 0x173 at p[6] and 0x12F at p[5]. It should read **p[7]** and **p[6]**. So it shows "no change" on PT-CAN while really both toggle 3 times. Its FlexRay 0x74 check (p[6]) is correct. Fix before v1g runs.

### 4.2 Brake pressure / pedal signals that rise with each press

All of these are zero (or at their offset) when released, and show three increasing peaks.

| Signal | Location | Scaling | Press 1 peak | Press 2 peak | Press 3 peak |
|---|---|---|---|---|---|
| **Brake pressure (best candidate)** | **FlexRay 0x3F**, cycle 1 mod 4 (every 20 ms), **bits 108–119** (12 bits: upper half of byte 14 + byte 15, 1-based) | raw ÷ 16 = bar (guess; ~1 bar per 16 counts). Exactly proportional to brake torque at **≈ 85 Nm per bar** | raw 214 = **13.4 bar** @ 142.82 | raw 557 = **34.8 bar** @ 150.34 | raw 1310 = **81.9 bar** @ 159.62 |
| **Pedal travel (likely)** | FlexRay 0x3F, cycle 1, **bits 96–107** (byte 13 + lower half of byte 14, 1-based) | Unknown unit. It rises fast at first and then flattens (5 bar ≈ 215, 20 bar ≈ 356, 40 bar ≈ 500, 80 bar ≈ 729), which is how pedal travel behaves against pressure | 303 @ 142.82 | 465 @ 150.34 | 739 @ 159.62 |
| Brake torque, total | PT-CAN **0x0EF** d2..3 and d5..6 (0-based, little-endian) = FlexRay 0x2B cycle 3; also FR 0x3F cycle 1 bytes 11–12 | raw − 32000 = Nm (negative = braking) | **−1134 Nm** @ 143.05 | **−2952 Nm** @ 150.35 | **−6959 Nm** @ 159.59 |
| Brake torque per wheel (likely) | FlexRay 0x2C cycle 3, four 16-bit fields at bytes 3–4, 5–6, 7–8, 9–10 (1-based) | raw − 32000 = Nm | −185 / −185 / −381 / −381 | −484 / −484 / −993 / −991 | −1141 / −1141 / −2339 / −2339 |
| Coarse brake level | PT-CAN 0x25D d4 (0-based), upper 4 bits (low 4 bits stay 0xB) | steps 0 … 6 | 2 | 3 | 6 |

**About the per-wheel row:**
- The four values add up to the 0x0EF total (2×185 + 2×381 = 1132).
- One axle (bytes 7–10) carries about twice the torque of the other (bytes 3–6). That fits front vs rear.

**Timing of press 3 (maximum), as an example:**

| Event | t (s) |
|---|---|
| switch on | 155.563 |
| pressure starts | 155.678 |
| reaches 50 bar | ~157.15 |
| peak 81.9 bar | 159.62 |
| held 76–81 bar | until ~161.3 |
| back to 0 | 163.10 |
| switch off | 163.17 |

**Press start/end times:**

| Press | Pressure first > 0 | Pressure back to 0 |
|---|---|---|
| 1 | 141.618 | 144.818 |
| 2 | 146.978 | 151.538 |
| 3 | 155.678 | 163.097 |

**Notes:**
- The colleague's note in `dde_brake_rx.md` says "no pressure, no pedal-travel signal". That holds for the DDE's own receive fields: FR 0x3F bits 32–55 stay at "not available". **But the same FlexRay 0x3F frame does carry pressure-like and travel-like fields further along (bits 96–119).** Whether the DDE reads them is not known.
- About 13 % of frames were lost on USB (§3), so a single peak sample may be missed. The signals are smooth, so the peaks above are accurate to within a sample or two.

**Data files:**
- `brake_peaks_run7.csv`: every candidate, per press.
- `brake_press_summary_run7.csv`: pressure, travel and torque peaks and start/end times.
- `brake_timeseries_run7.csv` and `brake_fr3f_timeseries_run7.csv`: the raw curves from 138–166 s.

---

## 5. Comparison with run 6 (v1e)

| Item | Run 6 (v1e) | Run 7 (v1f) |
|---|---|---|
| 20 ms frames | 244 per 5 s (2.5 % short) | 250–256 per 5 s (on time) |
| 0x397 spacing | fixed 1000 ms | ~400–490 ms average (inside the 250–600 ms target) |
| ST_KL on/off | A → 7 → 6 | A → 7 → 6 |
| Sleep after OFF | +52.67 s CAN / +52.77 s FR | +52.63 s / +52.72 s |
| New messages | none | only diagnostic 0x62A/0x6F0 (harmless) and key-related CC 66/652 in the dead-key wake |
| USB drops | 0 | **187 933 (13 %)**, a regression |
| Own Tx in capture | not supported | supported by firmware, **not recorded** (old logger used) |

## 6. To do for the next run (v1g)

1. Use the logger that ships with the firmware and check that the banner mentions "dir=Tx".
2. Fix the per-loop transmit-event read (§3) and confirm `usb_fifo_dropped = 0` on the bench first.
3. Fix the checker's 0x173 / 0x12F byte indexes, and make it fail on USB drops.
4. With Tx rows present, re-check: ≥ 99.5 % slot coverage, 0x397 content and blips, exact 0x518/0x512 last-frame times, and the 0x592 content.
5. Optional: one very slow, light brake press, to find out which switch bit is the brake-light switch.
