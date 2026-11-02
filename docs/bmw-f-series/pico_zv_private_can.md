# Pico ↔ ZombieVerter on F11 PT-CAN, draft 2: the ZV sends every BMW frame
> **v1z.1 copy (pico-flexray docs/zv_link/).** One addition to draft 2: 0x308 PICO_Status bit 31 `MIRROR_SRC_PTCAN` (1 = the FlexRay state is mirrored from the ZV's BMW PT-CAN frames, because PR #276 does not send 0x304-0x307 yet; 0 = from the private frames, or not mirrored). The v1z.1 firmware implements the Pico side of §6/§8 (src/zv_link.c); §8 calls it "v1i", it is built as v1z.1 (not yet run on a car).

> **Copy in this repo (docs/bmw-f-series).** This is the Pico v1z.1 copy of draft 2. The ZV code that implements it (PR #277, `src/vehicles/BMW_F11.cpp`) differs from the text below in these points:
> - The second-sender canary (§3.6, and the 0x3F9 / 0x0A5 / 0x3FD / 0x304 receive filters for it) is **not** in the ZV. Damien chose a clean split instead: the Pico never sends BMW frames. `F11Fault` 1 now means "Pico lost".
> - The ZV sends 0x304-0x307 and listens to 0x308-0x30C only when the parameter `F11Pico` = 1 (default 0). PR #277 does send 0x304-0x307, so the "PR #276 does not send them yet" remark above is out of date.
> - Where draft 2 and the Pico v1z.1 firmware disagree, the ZV follows v1z.1: 0x308 bit 31 `MIRROR_SRC_PTCAN`; FB7_MV is left out of the 0x307 change check; 0x30C is only CRC/alive-checked and timed (no consistency monitor); `FLT_WDT` / `FLT_OVERRIDE` are never set; on a manual box 0x306 shows the gear set from W+709 ms as v1z.1's mirror does.
> - Decoded Pico data (DSC, brake, cruise) is shown as values only; the ZV does not act on it.

Status: **design only**. Nothing was written to either firmware, nothing was flashed or pushed, and nothing was sent on any bus.

Companion files in this folder:
- `pico_zv_private_can.dbc`: the private frames of §6, draft 2. It checks clean with cantools, with no overlapping bits.
- `pico_zv_ptcan_ids_seen.csv`: every PT-CAN ID in our captures. Draft 2 adds two columns:
  - `tx_owner_draft2`: who sends the ID now;
  - `zv_status_vs_v1h3`: the gap status.
  - It also adds the 9 planned private IDs.
- Draft 1, where the Pico owned the BMW frames, is kept at `/workspace/scratch/pzv/pico_zv_private_can.draft1.md`.

Sources:
- **Pico:** v1h.3, `pico-flexray` branch `dde-emu-v1h`, commit **63eb320**, 2026-11-01 14:04 GMT, checked out at `/workspace/scratch/emu-v1f`.
- **ZV:** PR #275, `damienmaguire/Stm32-vcu`, branch `cursor/bmw-f11-ptcan-5b2a`, head **0b7db60**, 2026-11-01 11:31 GMT, class `BMW_F11`.

Line references:
- `core.c:N` = `src/ptcan_emu_core.c`
- `eng.c:N` = `src/dde_engine.c`, `eng.h:N` = `src/dde_engine.h`
- `gear.c:N` / `gear.h:N` = `src/dde_gear.c` / `.h`
- `emu.c:N` = `src/dde_emu.c`
- `F11.cpp ~N` = PR #275 `src/BMW_F11.cpp`, as fetched at the branch head (± 2 lines).

---

## 0. In plain English (for Damien)

* **Who sends what.**
  * **The ZV becomes the only box that sends BMW frames on PT-CAN.** That is the 19 DDE/EGS frames the Pico sends today.
  * **The Pico sends nothing BMW on PT-CAN.** It keeps doing the DDE/EGS job on FlexRay, and it talks to the ZV only on the private IDs 0x304–0x30F. The block stays as chosen: no capture, no BMW list and no other bus uses it.
* **What the ZV is missing.** PR #275 already sends 9 of the 19 frames.
  * 3 are right or close enough: 0x0D9, 0x0F3 and 0x2C4 differ only in small details.
  * 6 need fixing: 0x3F9, 0x0A5, 0x0DC, 0x08F and 0x592 (timing), plus 0x281, where real voltage is fine.
  * **10 frames are missing:** 0x3FD, 0x39A, 0x396, 0x518, 0x512, 0x1B9, 0x381, 0x3B3, 0x397 and 0x3FB.
* **The worst current bug: 0x3F9 always says "P secured".**
  * The PR forces 0x3F9 bits 28–29 = 2 ("P secured") at all times, even in D. The CAS reads those bits: in run 6 it reacted within 4 ms.
  * The PR also has no gear logic. Gear comes from the ZV's own shifter settings, and the F30_Lever class would send its own 0x3FD.
* **The simplest correct way to build it.** The Pico code that makes these frames is plain C with no Pico dependencies: `ptcan_emu_core.c`, `dde_engine.c` and `dde_gear.c`. These are the frame timelines, the fake engine start/stop and the gear lever logic, all proven in the car in runs 9 and 10.
  * **Recommendation: copy those three files into Stm32-vcu and drive them from `BMW_F11`.** Use `Task1Ms` as the scheduler, with a 1 ms tick as on the Pico. Then the ZV makes byte-for-byte what the car already accepted.
  * Do not re-type the logic into new C++. That is how today's differences crept in.
* **Gear logic belongs in the ZV.**
  * The ZV reads the lever (0x197) and the brake (0x173) directly, and sends 0x3FD/0x39A/0x3F9/0x0DC itself. That removes any CAN hop between the lever and the gear display that the GWS checks.
  * The Pico only has to copy two gear bits and the gear nibble into FlexRay 0xE6, and two bytes into FlexRay 0x28. The ZV sends those in `ZV_EngState` (0x306).
* **The engine state belongs in the ZV too**: starting, running and stopping, rpm and torque. It goes to the Pico as the exact values to put on FlexRay (0x305/0x306/0x307). The Pico no longer runs its own engine story; it mirrors.
  * Both buses then carry the same numbers. FlexRay 0xE6 lags PT-CAN 0x3F9 by about 2–16 ms. The real DDE lags by 9–12 ms (§7), so this is fine.
* **One real timing risk: the start.**
  * The PR only says "engine running" (0x82) once the inverter is in Run, which is after precharge.
  * The car's CAS holds the starter request (ST_KL D) until it sees 0x82. How long the CAS waits before giving up was never measured. Real cranks take 0.8–1.2 s; our accepted fake start reports 0x82 at D + 176 ms.
  * **Recommendation:** play the proven SHORT fake start profile (0x82 at about D + 172 ms) regardless of precharge. Allow torque only once the inverter really is in Run. Decision needed (§11).
* **The Pico firmware needs a small new mode:** "PT-CAN BMW TX off, private link only". Its foreign-sender check is moved onto its own private IDs, and its engine/gear values come from the ZV instead of its own state machines (§8).
* **Bus load**:
  * The BMW frames do not change, only who sends them: about 300 frames/s, 6–7 %.
  * The private link adds about 402 frames/s: **9.2 %** typical, 10.9 % worst case.
  * PT-CAN would then be about **28–29 % mean and about 33 % peak**. That is fine at 500 kbit/s.
* **The ZV must stay powered for about 50 s after the ignition goes off.**
  * Network management (0x512) runs to OFF + 46 s, 0x3FD shows P to OFF + 30.2 s, and the last MIL-off 0x592 goes at OFF + 54 ms.
  * If the ZV is powered from KL15, these all stop early. Also, a wake without ignition (door unlock) expects the DDE/EGS frames for 3–30 s.

---

## 1. Who sends what

| What | Sender | Receiver | Notes |
|---|---|---|---|
| The 19 DDE/EGS PT-CAN frames (§5) | **ZV** | car | Was the Pico (v1h.3). Exactly one sender per ID. |
| 0x202 (F30_Lever) | **nobody** | | Remove: no BMW shifter class may run with BMW_F11 (§5.20) |
| 0x304–0x307 private (§6.1) | ZV | Pico | ZV heartbeat + FlexRay mirror values. Uses the spare 0x307 too. |
| 0x308–0x30C private (§6.2) | Pico | ZV | Pico heartbeat, FR ST_KL, FR-only DSC/brake/cruise inputs, FR echo. 0x30D–0x30F spare. |
| DDE/EGS FlexRay emulation | Pico | car | Unchanged, except that the engine/gear overlay comes from the ZV (§7) |
| 0x12F, 0x173, 0x0EF, 0x197, foreign NM 0x500–0x57F | car | ZV (and Pico, logging) | ZV inputs (§3.1) |

The **ID block stays 0x304–0x30F**:
- the reasoning of draft 1 §3 is unchanged;
- the split stays ZV→Pico 0x304–0x307 / Pico→ZV 0x308–0x30F, so each side can use one mask filter (0x304/0x7FC and 0x308/0x7F8).

Draft 1 had 4 Pico→ZV frames that the Pico no longer needs (the gear frame 0x30B is dropped). The ZV now needs 4 frames to feed FlexRay, which is why 0x307 is used.

---

## 2. ZV gap list (short)

The "PR #275" column is the PR head as fetched.

| ID | v1h.3 period | PR #275 | What to do in BMW_F11 |
|---|---|---|---|
| 0x3F9 | 1 s + event (gap 1 ms) | **different** | Wake ramp (d3/d6). States 81 (crank)/82/92 (stopping)/80 with start-ready 2→1→0→1, b4 0x65/0x26/0x66/0x64, crank marker ctr_hi 3. Park bits from the gear logic, not a forced 2. Event gap 1 ms, not 40 ms. First frame at W+439 ms. |
| 0x3FD | 100 ms + gear event (gap 5 ms) | **missing** | New. `CC ctr 20/00 fc ff`, gear display from the gear logic, P hold to OFF+30.164 s. F30_Lever must be off. |
| 0x39A | 1 s + event (gap 5 ms) | **missing** | New. Wake ramp `e3`→`e2`→`00 35 00 03 e2 00`. d3/d4 from the gear logic. |
| 0x396 | 1 s | **missing** | New, constant `fd ff` |
| 0x518 (EGS NM) | 645 ms | **missing** | New. NM content/timing per KL15, release OFF+30 s, ceiling 60 s |
| 0x0A5 | 20 ms | **different** | Torque raw 0xA1A with KL15 (0x7B8 before KL15, 0xA19 after), not 2047. Fake-start torque/rpm profile. First frame at W+459. |
| 0x0D9 | 20 ms | different (minor) | Bytes OK at 0 % pedal. Add ctr_hi 3 during the crank marker. Decide on the real pedal (v1h.3 = 0 %). |
| 0x0DC | 20 ms | **different** | d5 = F4 in R and D (PR: FE always). P-coming `22 .. FC`. Wake ramp `80 ff ff f0` → `20 d0 07 f1` → `22 d0 07 fe`. |
| 0x0F3 | 20 ms | different (minor) | Same bytes. rpm/10 should lag by about 200 ms; use the engine-story rpm. |
| 0x08F | 30 ms | **different** | d4 0x8C until the first run in this wake (PR 0x7C). d2–3 `F9 7C` after first rpm + 100 ms. d7 80 (pre-KL15) / 10 (ON+15 ms) / C0 (OFF+31 ms). |
| 0x2C4 | 100 ms | different (minor) | d0–1 fuel counter that integrates rpm (PR 0) |
| 0x281 | 100 ms | different (OK) | Real 12 V is acceptable. Keep the v1h.3 crank dip 0x2C1 if you want parity. |
| 0x1B9 | 1 s | **missing** | New, constant `00 ff` |
| 0x381 | 10 s + event (gap 20 ms) | **missing** | New, 4-value timeline + `79 f0` while running |
| 0x3B3 | 5 s + event (gap 20 ms) | **missing** | New, 3 contents + the engine FB9 bytes |
| 0x397 | 1 s + event (gap 15 ms) | **missing** | New. Wake ramp, e3 blip table, off walk, engine overlay. |
| 0x3FB | 1 s + event (gap 60 ms) | **missing** | New. Ramp d1/d5, d4 bit 0 while "injecting". |
| 0x512 (DDE NM) | 645 ms | **missing** | New. 4 pre-KL15 frames, ON+106 ms, release OFF+46 s, ceiling 60 s. |
| 0x592 | 5.1 s while KL15 + final | **different** | MIL-off from ON+98 ms every 5.1 s while KL15, one final frame at OFF+54 ms (PR: once at Run) |
| bus gate | — | **different** | Stop all BMW TX 200 ms after the car goes quiet (PR: 2 s). New wake after ≥ 1 s of silence. Prep-sleep rule. NM ceiling. |
| gear logic | — | **missing** | Port `dde_gear.c` (lever 0x197, brake 0x173, 40 ms/20 ms/10 ms/900 ms/340 ms/3 s timing) |
| engine story | — | **missing** | Port `dde_engine.c`: CAS mode, SHORT profile, stop triggers, run-down |
| private link | — | **missing** | TX 0x304–0x307, RX 0x308–0x30C (§6) |

---

## 3. Common ZV machinery

This is what every frame in §5 depends on.

### 3.1 Inputs, filters, bus gate and timelines (port `core.c:115–245, 480–500`)

**RX registrations in `BMW_F11::SetCanInterface`:**
- `0x12F`, `0x173`, `0x0EF`, as today;
- **new:** `0x197` (lever);
- **new:** `0x500` mask `0x780` (foreign NM 0x500–0x57F, for the prep-sleep rule);
- **new:** `0x304` mask `0x7FC` and `0x308` mask `0x7F8`. These are the private link and the foreign-sender canary (§3.6).
- **new canaries:** `0x3F9`, `0x0A5`, `0x3FD`. A bxCAN node never receives its own frames, so any of these arriving means a second sender.

Notes on capacity:
- `RegisterUserMessage(id, mask)` takes a mask in current libopeninv: `canhardware.h:66`, with `MAX_USER_MESSAGES` 30 shared by every module on that interface. Check the Stm32-vcu submodule pin.
- The software TX queue is `SENDBUFFER_LEN` 20 (`stm32_can.h:25`). A full queue silently drops the frame, so count drops (§3.4).

**Times.** As in `core.c:20–24`:
- **W** = the first PT-CAN frame after ≥ 1 s of silence (`PTE_NEW_WAKE_US`). At W, all frame states and counters reset (`new_wake`, `core.c:158–172`), so the first E2E frame carries alive 1.
- **ON** = the first 0x12F with ST_KL ≥ 0xA.
- **OFF** = the first 0x12F with ST_KL < 9 after ON.

The hysteresis is `pte_kl15_from_stkl` (`core.c:115–122`): ≥ 0xE invalid (ignore), ≥ 0xA on, 9 holds, ≤ 8 off.

**"Bus alive" for TX.**
- v1h.3 counts *any* received frame (`core.c:211`).
- The ZV only hears its registered IDs. Use **0x0EF (20 ms), 0x12F (100 ms, 20 ms in C/D) and 0x173 (100 ms)**, plus the NM mask. The ZV must ignore its own canary IDs and 0x304–0x30F for this rule.

**States.** Port `pte_state()` exactly (`core.c:480–494`):
- `RUN`: send.
- `SILENT`: no alive frame for **200 ms** (`PTE_SILENCE_US`). Stop all BMW TX at once.
- `WAIT_BUS`: ≥ 1 s of silence. The next frame is a new W.
- `PREP_SLEEP`: KL15 off, own NM no longer active (§5.5/5.18), and no foreign NM for **1.95 s** (`PTE_SLEEP_PREP_US`). Stop all cyclics.
- `FAULT`: §3.6.
- PR #275 (`F11.cpp ~576`, `busQuiet100 < 20`) keeps sending for up to **2 s** after the car is quiet. That can wake a sleeping bus again. Replace it.

**Catch-up** (`core.c:595–603`, late counter `core.c:607–611`):
- Keep each frame's grid. A late slot still goes out late, unless it is more than 250 ms behind; then restart the grid from now and count a resync.
- A send more than 5 ms after its slot counts as "late".

### 3.2 E2E (BMW frames)

Same as v1h.3 `pte_e2e_crc` / `pte_next` (`core.c:91–95, 561–566`) and PR `E2ESeal` (`F11.cpp ~53`), which agree:
- `b1 = (ctr_hi << 4) | alive`;
- alive is 1..14 then 0, i.e. 0..14 and never 15. It goes +1 for **every** frame, event copies included;
- `b0 = CRC8(poly 0x1D, init 0)` over `[dataID lo, dataID hi, b1 .. b(dlc−1)]`.
- Verified on a stock 0x3FD frame: data ID 0x39 0x00 gives CRC 0xBC.
- Engine overlays may change `ctr_hi` (0x0D9/0x3F9 crank marker, 0x0F3 rpm nibble) **before** the CRC is computed.

### 3.3 Scheduler (port `pte_next`/`pte_commit`, `core.c:502–622`)

Call it from `BMW_F11::Task1Ms()` (`Vehicle::Task1Ms` exists, `vehicle.h:37`; `stm32_vcu.cpp:875–877`). Each pass:
1. Return if the state is not RUN.
2. For each enabled frame, `frame_active()` (`core.c:277–297`). Then `build()` (`core.c:300–478`). Then the engine/gear overlay `eng_patch_can()` (`eng.c:644–667`).
3. Due time:
   - unstarted frame: `W + first_ms`;
   - NM frames: `next_us`;
   - CYC_EVENT kinds (and 0x3FD while `g_set`): if the new payload differs from the last **intended** payload `want` (v1h.2 fix, `core.c:536–541`), mark an event pending. Send it now if at least `ev_gap` has passed since the last TX and the cyclic slot is not already due.
4. Send the most overdue frame first. Seal the counter and CRC. Commit:
   - event copies do not move the grid;
   - NM frames keep a 645 ms grid;
   - 0x592 is `now + 5.1 s`.
5. Send up to 4 frames per 1 ms pass (`PTCAN_BURST`). At most 3 bxCAN mailboxes plus the software queue are needed.

**Do not call the scheduler from the 10 ms task.** The event gaps (1/5/15/20 ms), the gear lags (10/20/21 ms) and the first-frame offsets need ~1 ms resolution. A 10 ms tick moves them by up to 10 ms, and the GWS/CAS timings in §5 were tuned to ±1–2 ms in run 10.

### 3.4 Frame table (port `pte_defs`, `core.c:52–73`)

| # | ID | DLC | kind | E2E data ID | ctr_hi | period ms | first ms after W | event gap ms | ECU |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 0x3F9 | 8 | cyclic + event | 0x0037 | 1 | 1000 | 439 | 1 | DDE |
| 2 | 0x3FD | 5 | cyclic (+ gear event) | 0x0039 | 0 | 100 | 462 | 5 (gear) | EGS |
| 3 | 0x39A | 6 | cyclic + event | — | — | 1000 | 462 | 5 | EGS |
| 4 | 0x396 | 2 | cyclic | — | — | 1000 | 600 | — | EGS |
| 5 | 0x518 | 8 | NM (EGS) | — | — | 645 | 543 | — | EGS |
| 6 | 0x0A5 | 8 | cyclic | 0x0009 | F | 20 | 459 | — | DDE |
| 7 | 0x0D9 | 8 | cyclic | 0x0043 | 1 | 20 | 462 | — | DDE |
| 8 | 0x0DC | 6 | cyclic | 0x005A | 1 | 20 | 462 | — | DDE |
| 9 | 0x0F3 | 8 | cyclic | 0x00A6 | 0 (rpm nibble) | 20 | 462 | — | DDE |
| 10 | 0x08F | 8 | cyclic | 0x0020 | 1 | 30 | 459 | — | DDE |
| 11 | 0x2C4 | 8 | cyclic | — | — | 100 | 611 | — | DDE |
| 12 | 0x281 | 2 | cyclic | — | — | 100 | 612 | — | DDE |
| 13 | 0x1B9 | 2 | cyclic | — | — | 1000 | 611 | — | DDE |
| 14 | 0x381 | 2 | cyclic + event | — | — | 10000 | 435 | 20 | DDE |
| 15 | 0x3B3 | 7 | cyclic + event | — | — | 5000 | 437 | 20 | DDE |
| 16 | 0x397 | 3 | cyclic + event | — | — | 1000 | 437 | 15 | DDE |
| 17 | 0x3FB | 6 | cyclic + event | — | — | 1000 | 437 | 60 | DDE |
| 18 | 0x512 | 8 | NM (DDE) | — | — | 645 | 529 | — | DDE |
| 19 | 0x592 | 8 | check-control | — | — | 5100 | ON+98 | — | DDE |

Keep the forbidden-ID belt-and-braces check (`pte_id_forbidden`, `core.c:98–113`): 0x6xx, 0x137, 0x27A, 0x197, 0x1AF and the rest of that list.

### 3.5 Engine story (port `dde_engine.c` whole; spec `eng.c:56–480`)

The ZV owns the fake engine. It produces one `eng_out_t` (`eng.h:94–134`) per step. That struct drives the PT-CAN overlays (§5) **and** the private frames 0x305–0x307 (§6.1).

**Inputs:**
- `eng_input_stkl(st_kl = 0x12F d2 & 0x0F, brake = (0x12F d6 & 3) == 3, t)` on every 0x12F (`emu.c:1541–1547`).
- Optionally the same from `PICO_Status.FR_STKL`, which arrives ~7 ms earlier (§7.3).
- `eng_input_brake(true, t)` on every 0x173 with `(d7 & 3) == 3` (`emu.c:1557`).

**Mode and profile:**
- Mode `ENG_MODE_CAS` permanently. Call `eng_set_mode(CAS, now)` every step: the ZV replaces the PC keepalive of `eng.c:470–471`.
- Profile **SHORT** (`eng.c:31`), the build default accepted in run 10.

**Step:** `eng_step()` every **5 ms** (a `Task1Ms` divider; v1h.3 steps once per 5 ms FlexRay cycle). Then `gear_step()` + `gear_fill_out()` (§3.7) into the same `eng_out_t`.

**State machine (exact, from the code):**

| Phase | Entered when | Main outputs (eng_out_t) |
|---|---|---|
| OFF | wake; nothing yet | `active` 0 → no overlay; the frames use their templates |
| CRANK | `check_start` (`eng.c:215–234`): phase OFF/STOPPED, ST_KL == D fresh (≤ 300 ms), brake within 2 s | SHORT: rpm from t=0 (`rpm4_start`, `eng.c:134–158`: cranking 115–210 rpm until 50 ms, run-up 200→1000 rpm over 130 ms). `cnt_hi` 3 for t < 105 ms. `e6` 81/65, ready 1. Torque 2560 → falling. `fb7` 10600, `v281` 0x2C1. `inj` from 40 ms. |
| RUN | rpm ≥ 950 (`eng.c:292–294`; SHORT ≈ T0+172 ms) | `e6` 82/26, ready 0 (SHORT `ready_off` 0). Torque 2140 (to 1.5 s), then 2098. rpm flare 1000–1060 → idle 780 ± 1. r29_b5 72, f97c, fb9 ramps, e89 from +2.8 s, r381, r397 … (`eng.c:314–378`) |
| STOP | `check_stop` (`eng.c:236–261`): ST_KL ≤ 8; ST_KL 9 for 1 s; ST_KL stale 600 ms; SHORT start abandoned if D ended before 82 (`need_d`) | `e6` 82 → **92 at TS + t92 (SHORT 2 ms)**, b4 66, ready 0. rpm run-down `stop_tab` (`eng.c:162–172`) to 0 at 1.2 s. Torque 2098 → 1947. |
| STOPPED | rpm 0 and TS + 1200 ms | `e6` 80/64, ready 1 from +20 ms. Torque 1978. 0x397 walk to +2.913 s. `fb7` decays over 5 s. |

**What changes for the ZV, two decisions (§11):**
- **(a) "Never report running before START" vs precharge.**
  - v1h.3 reports 82 at about D + 172 ms from the profile alone.
  - The PR reports running only when `opmode == MOD_RUN` (`EngineRunning`, `F11.cpp ~432`).
  - Recommended: the engine story runs on the profile. Separately, the ZV allows torque only in MOD_RUN with the gear STEADY in R/D (§3.7). If precharge has not reached MOD_RUN within 5 s of the start, run `stop_now()` (0x92 → 80) and set MIL.
- **(b) rpm after RUN.**
  - Keep `rpm4_start()` until the flare ends at 1.5 s.
  - After that: `rpm4 = 4 × max(idle 780 ± 1 wobble, |motor rpm × ratio|)`, clamped to 8000. Here ratio = virtual engine/motor rpm (start with 1.0).
  - This replaces the PR's `RpmTx` 750..8000. 0x0F3 follows through `f3_val` (lagged).

### 3.6 Foreign-sender interlock (ZV side)

v1h.3 latches `PTE_FAULT_FOREIGN` when an enabled own ID arrives from another node (`core.c:216`). The ZV equivalent:
- **trigger:** any registered canary (0x3F9, 0x0A5, 0x3FD), or 0x512/0x518 from the NM mask, or any of 0x304–0x307;
- **action:** set `FOREIGN_FAULT`, stop all BMW TX and the private TX, and set ZV_Status.FAULT.

This is what catches a Pico still running a legacy v1h.3 build with PT-CAN TX on (§8.4), or a second ZV class sending.

A 0x304–0x307 frame is never received by the ZV itself, so if one arrives it is a foreign sender. Clear the fault only on a parameter write or a power cycle.

### 3.7 Gear logic (port `dde_gear.c` whole; spec `gear.c:24–308`, `gear.h:74–85`)

**Inputs:**
- `gear_input_lever(d, len, t)` on every 0x197 (`gear.c:171–212`). The decode, CRC check, notch and P-button handling are all in that function.
- `gear_input_brake(t)` on every 0x173 with `(d7 & 3) == 3` (`gear_tick`, `emu.c:875–881`).
- `gear_step(now, running = (eng.phase == ENG_RUN))` every 5 ms after `eng_step`. Then `gear_fill_out(&gear, &eng.out)`.

**Lever decode (0x197):**
- B0 is a CRC with data ID 0x3F 0x00 over 4 bytes.
- B2 >> 4 is the lever position: 0 centre, 1/2 forward 1/2 notches, 3/4 back 1/2 notches.
- B3 & 0x3F == 0x15 means the P button is pressed; 0x00 means released.

**Shift table** (`gear_act`, `gear.c:130–166`):
- From P: forward 1 → N, forward 2 → R, back → D.
- From R: back 1 → N, back 2 → D.
- From N: forward → R, back → D.
- From D: forward 1 → N, forward 2 → R.
- P button → P.

**Gating:**
- Leaving P, and P/N → R/D, needs running **and** brake: a pressed 0x173 in the last 250 ms.
- When the engine is not running: auto-P.
- Lever stale after 90 ms: hold the gear.

**Timing** (`gear.h:74–85`):
- act 40 ms after a notch first appears;
- P released at +20 ms;
- gear code 10 ms after 0x3FD (100 ms for N);
- P button: 0x39A 01 at +21 ms, 0x3F9 nibble 1 and 0xDC `22 .. FC` at +42 ms, 0x39A 03 at +881 ms, 0x3FD P at +900 ms;
- pawl "locked" 340 ms after brake release, or 3 s after the request at the latest.

**Output bytes** (`gb_gear`, `gear.c:32–38`):

| gear | 0x3FD d2 | 0x39A d3 | 0x39A d4 | park bits (0x3F9 d3 b4-5) | nibble (0x3F9 d6 lo) | 0x0DC d2 | 0x0DC d5 |
|---|---|---|---|---|---|---|---|
| P locked (no overlay, `g_set` 0) | 20 | 03 | E2 | 2 | 3 | 22 | FE |
| P unlocked | 20 | 03 | E0 | 0 | 3 | 22 | FE |
| R | 40 | 02 | E0 | 0 | 2 | 25 | F4 |
| N (hypothesis except 0x3FD) | 60 | 01 | E0 | 0 | 1 | 22 | FE |
| D | 80 | 05 | E0 | 0 | 5 | 24 | F4 |
| P released / leaving P | P or new | 03 | **E1** | **1** | 3 | 22 | FE |

`g_fr_park` (FlexRay 0xE6 b4 bits 4–5) stays **2** during P released / leaving P (`gear.c:307`).

**Drive direction:**
- Write `Param::dir` from `gear.target` when `stage == GST_STEADY`: R → −1, D → +1, P/N → 0. Also return it from `GetGear()` (`vehicle.h:46`).
- Allow torque only when `stage == STEADY` and the gear is R/D.
- This replaces `GearFromDir()` (`F11.cpp ~103`) and any shifter class.

**Safety note:** the 8HP is gone, so "P" is display only. Holding the car needs the EPB, or a ZV-side "P ⇒ zero torque + EPB request". Not part of this spec.

---

## 4. Contents rule used below

- `dN` = data byte N, 0-based. CC = CRC, AA = alive.
- "Template" = `build()` content (`core.c:300–478`).
- "Overlay" = `eng_patch_can()` (`eng.c:567–620` engine part, applied only if `eng_out.active`; `eng.c:644–667` gear part, applied only if `g_set`).
- v1h.3 applies **no** overlay when the engine story is inactive (`eng_snap_select`, `eng.c:670`). Templates alone then reproduce the stock "KL15 on, engine off, P" car.

---

## 5. Per-frame spec (what BMW_F11 must send)

### 5.1 0x3F9 DDE engine status (= FlexRay 0xE6). E2E 0x0037, ctr_hi 1. 1 s + event (gap 1 ms). First frame at W+439.

**Template** `CC 1A d2..d7` (`core.c:308–322`). Base `80 34 3c 30 00 47`, then:

| from W+ | change |
|---|---|
| 459 ms | d6 = 10 |
| 509 ms | d6 = 20 |
| 529 ms | d3 = 74 |
| 609 ms | d3 = 64 |
| 709 ms | d6 = 23 |
| 1379 ms | d5 = 43 |
| 3219 ms | d4 = 3d |
| 6379 ms | d5 = 3c |
| 7379 ms | d5 = 3d |

**Engine overlay** (`eng.c:579–583`):
- ctr_hi = 3 while `cnt_hi`;
- if `e6_b3`: d2 = e6_b3, d3 = e6_b4;
- if `e6_ready != 0xFF`: d6 high nibble = e6_ready.

**Gear overlay** (`eng.c:657–660`): d3 bits 4–5 = `g_park`, d6 low nibble = `g_nib`.

**Event:** any change of d2..d7 against the last intended payload.

**PR #275 differences** (`Send3F9`, `F11.cpp ~696–734`):
1. d3 bits 4–5 forced to 2. Wrong in R/N/D and in "P released".
2. Only 80/64 or 82/26 with ready 2/0. Never 81/65 ready 1 (crank), never 92/66 (stopping), never ready 1 after a stop.
3. No crank marker.
4. Minimum event gap 40 ms. v1h.3 uses 1 ms; the real 82 → ready 0 → D→C chain is only 5–20 ms.
5. No wake ramp: the first frame goes out at the first bus frame.
6. d4/d5 carry real temperatures. That is fine, but the Pico must copy them into FR 0xE6 (§6.1 `T3F9_D4/D5`).

### 5.2 0x3FD EGS gear display. E2E 0x0039, ctr_hi 0. 100 ms. First frame at W+462. Gear event gap 5 ms.

**Template** `CC 0A d2 fc ff` (`core.c:324–331`):
- d2 = 0x20 from ON+27 ms to OFF+30.164 s (`PTE_3FD_P_HOLD_US`);
- otherwise 0x00 (display off, also before any KL15).

**Gear overlay** (`eng.c:648–651`): d2 = `g_3fd`, **only while d2 ≠ 0**.

**Event:** while `g_set`, an extra copy on change, min gap 5 ms (`core.c:533–535`).

**PR:** missing. F30_Lever also sends a 0x3FD in another format and must not run.

### 5.3 0x39A EGS gear / P-lock. No E2E. 1 s + event (gap 5 ms). First frame at W+462.

**Template** (`core.c:332–340`):

| when | d0..d5 |
|---|---|
| W < 583 ms | `00 00 00 00 e3 00` |
| W < 680 ms | `00 00 00 00 e2 00` |
| after that | `00 35 00 03 e2 00` (d1 = ATF temperature + 40, constant) |

**Gear overlay** (`eng.c:653–655`): d3 = `g_39a_d3`, d4 = `g_39a_d4`.

**PR:** missing.

### 5.4 0x396 EGS. No E2E. 1 s. First frame at W+600.

Constant `fd ff` (`core.c:341–343`). **PR:** missing.

### 5.5 0x518 EGS NM. No E2E. 645 ms grid. First frame at W+543.

**Content** `00 00 00 00 d4 d5 00 18` (`core.c:344–352`):
- d4 d5 = `b0 30` for the first frame of a wake, with KL15 from ON+157 ms, and up to OFF+225 ms;
- otherwise `44 2f`.

**Active** (`frame_active`, `core.c:287–291`):
- while KL15;
- after KL15 until OFF+30.0 s;
- a wake without KL15: until W+30 s (`PTE_518_NOKL15_US`, an assumption: never captured);
- never later than 60 s after max(W, last 0x12F with KL15 on) (`PTE_NM_CEILING_US`).

The grid survives late slots (`core.c:583–584`).

**PR:** missing.

### 5.6 0x0A5 DDE torque/rpm. E2E 0x0009, ctr_hi F. 20 ms. First frame at W+459.

**Template d2..d7** (`core.c:353–362`):

| when | d2..d7 | torque raw |
|---|---|---|
| before KL15 | `b8 87 7b 00 00 f1` | 0x7B8 = −35.5 Nm |
| from ON+55 ms | `1a aa a1 00 00 f1` | 0xA1A = +269.5 Nm |
| after OFF | `19 9a a1 00 00 f1` | 0xA19 |

**Torque packing** (`put_torque`, `eng.c:483–488`):
- d2 = t & 0xFF;
- d3 = ((t >> 8) & 0xF) | ((t & 0xF) << 4);
- d4 = t >> 4.

The PR's packing is identical.

**Overlay** (`eng.c:571–574`): torque if ≠ 0; d5..d6 = `rpm4` LE.

**PR differences:**
- torque raw 2047 (0 Nm) always, deliberately (`F11.cpp ~601`);
- rpm 0 / 750..8000, with no crank or run-up profile.

**Recommendation:**
- use the v1h.3 templates while the engine story is inactive (proven);
- use the engine-story torque (crank 2560, idle 2140/2098, stop 2098/1947, stopped 1978) while active;
- once the ZV makes real torque, put `TorqueNmToRaw(actual)` there, and send the same raw to the Pico (`ZV_EngFast.TORQUE_RAW`).

### 5.7 0x0D9 DDE pedal. E2E 0x0043, ctr_hi 1. 20 ms. First frame at W+462.

**Template** `CC 1A 00 10 00 e0 7f c0` (`core.c:363–367`).

**Overlay:** ctr_hi = 3 while `cnt_hi` (`eng.c:576–578`).

**PR:** the bytes are identical at 0 % pedal (`PedalRaw`, `F11.cpp ~128`). It sends the real pedal, which v1h.3 never did, and it has no crank marker.

**Recommendation:** add the marker. Keep the real pedal behind a parameter that defaults to off, until a drive log shows nothing objects to it.

### 5.8 0x0DC DDE gear echo. E2E 0x005A, DLC 6, ctr_hi 1. 20 ms. First frame at W+462.

**Template d2..d5** (`core.c:368–374`):

| when | d2..d5 |
|---|---|
| W < 502 ms | `80 ff ff f0` |
| W < 703 ms | `20 d0 07 f1` |
| after that | `22 d0 07 fe` |

**Gear overlay** (`eng.c:661–663`): d2 = `g_dc_b2`, d5 = `g_dc_b5` (table §3.7; P-coming `22 .. FC`).

**PR:** d2 from `dir` (22/25/24), d5 always FE (wrong for R/D: F4), no P-coming state, no wake ramp.

### 5.9 0x0F3 DDE rpm/10. E2E 0x00A6, ctr_hi = rpm nibble. 20 ms. First frame at W+462.

**Template** `CC 0A 00 c0 f0 ff ff ff` (`core.c:375–379`).

**Overlay** (`eng.c:588–591`):
- ctr_hi = `f3_val & 0xF`;
- d2 = `f3_val >> 4`;
- `f3_val` = rpm/10 through a 200 ms first-order lag (`eng.c:440–446`).

**PR:** same bytes and nibble rule, but no lag.

### 5.10 0x08F DDE sensor/run (= FlexRay 0x29). E2E 0x0020, ctr_hi 1. 30 ms. First frame at W+459.

**Template** `CC 1A 00 7d 8c 00 18 d7` (`core.c:380–389`). d7:
- 0x80 before KL15;
- 0x10 from ON+15 ms;
- 0xC0 from OFF+31 ms.

**Overlay** (`eng.c:584–587`):
- `f97c` → d2..d3 = `F9 7C`, from first rpm + 100 ms, sticky for the wake;
- `r29_b5` → d4: 0x72 from first rpm + 7 ms, 0x7C from 92, 0x7C after a stop.

**PR** (`F11.cpp`, Task10Ms):
- d4 0x72 running, 0x7C otherwise. Wrong before the first run: it should be 0x8C;
- d2..d3 always `00 7D`;
- d7 always 0x10.

### 5.11 0x2C4 DDE fuel. No E2E. 100 ms. First frame at W+611.

**Template** `00 00 ff 64 64 64 00 f0` (`core.c:390–394`).

**Overlay:** d0..d1 = `fuel` (`eng.c:592–594`), a counter that integrates rpm: about rpm/20 per 100 ms, rpm/10 while cranking (`eng.c:448–454`).

**PR:** constant (fuel 0). Low priority.

### 5.12 0x281 DDE supply voltage. No E2E, DLC 2. 100 ms. First frame at W+612.

**Template** (`core.c:395–401`): d1 = 0xF3. d0:

| when | d0 |
|---|---|
| W < 811 ms, no KL15 yet | 0x51 |
| before KL15 | 0x4E |
| KL15 on: until ON+100 ms | 0x4E |
| KL15 on: from ON+100 ms | 0x33 |
| after OFF: until OFF+100 ms | 0x33 |
| after OFF: from OFF+100 ms | 0x36 |

**Overlay** `v281` (`eng.c:595–598`): 0x2C1 while cranking, ramping to 0x339 over 1 s after 82. d1 = 0xF0 | high nibble.

**PR:** real `uaux / 0.015`, or 0x356. Acceptable. Optionally force 0x2C1 during CRANK.

### 5.13 0x1B9 DDE. No E2E. 1 s. First frame at W+611.

Constant `00 ff` (`core.c:402–404`). **PR:** missing.

### 5.14 0x381 DDE. No E2E. 10 s + event (gap 20 ms). First frame at W+435.

**Template** (`core.c:405–419`):

| when | d0 d1 |
|---|---|
| no KL15 yet: W < 1411 ms | `0c f0` |
| no KL15 yet: after that | `7c f0` |
| KL15: ON < 867 ms | `7c f0` |
| KL15: ON < 3867 ms | `79 f0` |
| KL15: after that | `0c f2` |
| after OFF: < 423 ms | `0c f2` |
| after OFF: later | `7c f0` |

**Overlay:** `r381_run` → `79 f0` (`eng.c:599–602`).

**PR:** missing.

### 5.15 0x3B3 DDE (= FlexRay 0xFB L9). No E2E, DLC 7. 5 s + event (gap 20 ms). First frame at W+437.

**Template** (`core.c:420–427`):

| when | d0..d6 |
|---|---|
| W < 561 ms | `0f 00 00 00 00 00 f8` |
| KL15 from ON+79 ms, and after OFF until OFF+234 ms | `1f 00 04 00 00 03 f8` |
| otherwise | `1f 00 ff 7f 00 03 f8` |

**Overlay:** `fb9_set` → d1..d3 = `fb9_b2/b3/b4` (`eng.c:603–606`).

**PR:** missing.

### 5.16 0x397 DDE. No E2E, DLC 3. 1 s + event (gap 15 ms). First frame at W+437.

**Template** (`core.c:428–455`): d0 = c0, d2 = 00. d1:
- **wake ramp:** fe (W < 461), e8 (< 481), e9 (< 541), e6 (< 601), e4 (< 661), then e2;
- **"e3" blips:** first at W+1.321 s, then the 62-entry gap table `blip397_ms[]` (`core.c:253–259`, repeating). Each blip lasts 60 ms (gap − 20 ms if the gap is < 80 ms);
- **after OFF, when the engine story is inactive:** walk {93 df, 153 de, 273 dd, 373 de, 434 dd, 513 de, 634 dd, 713 de, 773 dd, 4073 df} ms, e2 before 93 ms, back to e2 at OFF+4.133 s.

**Overlay** (`eng.c:607–613`): `r397_set` → d0..d2 = r397. After `abs_stop_walk`, d0 = D0.

Copy the blip table verbatim from `core.c:253–259`.

**PR:** missing.

### 5.17 0x3FB DDE (→ FlexRay 0xE9 via the gateway). No E2E, DLC 6. 1 s + event (gap 60 ms). First frame at W+437.

**Template** `d0 r 01 10 f1 r/2` (`core.c:456–463`): r = (W − 440 ms) × 31/s, capped at 0xB1.

**Overlay:** `inj` → d4 &= 0xFE (`eng.c:607–610`). SHORT: from T0+40 ms; it ends 360 ms after the stop.

**PR:** missing.

### 5.18 0x512 DDE NM. No E2E. 645 ms grid.

**Content** `00 00 00 00 ff ff 00 12` (`core.c:464–468`).

**Timing** (`core.c:76–80, 225–236, 282–286, 580–585`):

| situation | frames |
|---|---|
| wake without KL15 | 4 frames at W+529, 1179, 2459, 3109 ms, then quiet |
| KL15 on | first at ON+106 ms, then a 645 ms grid |
| after OFF | until OFF+46.0 s |
| always | ceiling 60 s after max(W, last 0x12F with KL15 on) |

**PR:** missing.

### 5.19 0x592 DDE check-control (MIL off). No E2E.

**Content** `40 22 00 14 ff ff ff ff` (`core.c:469–474`).

**Timing:**
- first at ON+98 ms, then every 5.1 s while KL15;
- one final frame at OFF+54 ms (`CC592_*`, `core.c:79–80, 234–242, 587–590`).

**PR:** once when entering Run (`SendMilOff`, `F11.cpp ~741–753`). Use the v1h.3 timing (run 10: no MIL), and send MIL-on (`.. 15 ..`) only from a real ZV fault.

### 5.20 Remove: F30_Lever 0x3FD / 0x202

With BMW_F11 selected, force the shifter to "none". Otherwise two senders of 0x3FD trip the ZV's own canary (§3.6) and confuse the GWS.

---

## 6. Private link frames (implement in BMW_F11; DBC = `pico_zv_private_can.dbc`)

**Rules for all 9 frames:**
- DLC 8, Intel byte order.
- b0 = CRC8 (poly 0x1D, init 0) over `[ID lo, ID hi, b1..b7]`. The data ID is the CAN ID, e.g. `06 03` for 0x306.
- b1 low nibble = alive 0..14, +1 per frame including event copies. The first frame after a link (re)start = 1.
- A receiver **drops** a frame with a bad CRC, or with an alive equal to the previous one (a repeat). It counts these.
- Both sides send only while their own bus gate is RUN (§3.1, same rule on the Pico, §8.2). Neither side's frames count as "car traffic" for the other's gate.

### 6.1 ZV → Pico (ZV TX)

| ID | Name | Period | Event | Content (eng_out_t field → signal) | Pico timeout |
|---|---|---|---|---|---|
| 0x304 | ZV_Status | 20 ms | — | OPMODE, HV_READY, START_SEEN, DRIVE_EN, DERATE, FAULT, **MIRROR_EN**, MIL_ON, PICO_LINK_OK, **T3F9_D4/D5** (bytes exactly as in 0x3F9 d4/d5), BMW_TX_ON, ZV_BUS_STATE, FOREIGN_FAULT, FW_VER, TX_LATE | 100 ms → fallback |
| 0x305 | ZV_EngFast | 20 ms | on change of ACTIVE / CNT_HI3 | ACTIVE=`active`, CNT_HI3=`cnt_hi==3`, E8_FAST, E8_CRANK, RPM4=`rpm4`, TORQUE_RAW=`torque`, C3_B34, C3_B56, TURBINE=`turbine` | 100 ms |
| 0x306 | ZV_EngState | 20 ms | **on any change of b2..b7, gap 1 ms** | G_SET, F97C, E89, E6_B3, E6_B4 (engine value, before park bits), E6_READY (15 = none), G_NIB, G_DC_B2, G_DC_B5, G_FR_PARK, PHASE, GEAR | 100 ms |
| 0x307 | ZV_EngAux | 100 ms | on any change, gap 5 ms | FB9_SET, FB9_B2..B4, FB7_MV, E89_B1 code, B142_B8 code, B142_B23, R29_B5 code, R29_B8 code | 300 ms |

**Encoding notes:**
- Codes use the value tables in the DBC:
  - E89_B1: 0 = 12, 1 = 18, 2 = 1E, 3 = 25;
  - B142_B8: 1 = 84, 2 = A4;
  - R29_B5: 1 = 72, 2 = 7C;
  - R29_B8: 1 = 10, 2 = C0.
- These are the only values `set_outputs()` produces (`eng.c:264–466`). The ZV asserts on anything else.
- The FlexRay 0x28 c1 rpm/torque slot is every 4th cycle = 20 ms (`c4 == 1`, `eng.c:497–500`), so a 20 ms ZV_EngFast is enough.
- Send ZV_EngFast and ZV_EngState from the **same** 5 ms engine step, back to back (0x305 first), so the Pico never mixes two steps.

**What the Pico does with them** (§8.3): it rebuilds an `eng_out_t` and publishes it exactly where `engine_publish()` does today (`emu.c:898–931`). The FlexRay builders `eng_patch_fr_static` / `eng_patch_fr_dyn` (`eng.c:626–642`) stay unchanged. Plus one new patch: E6 b5/b6 = T3F9_D4/D5.

### 6.2 Pico → ZV (ZV RX)

| ID | Name | Period | Event | Content | ZV timeout → action |
|---|---|---|---|---|---|
| 0x308 | PICO_Status | 20 ms | on change of FR_STKL | PICO_MODE, **FR_STKL** (FR 0x74), FR_KL15, FR_SYNC, FR_TX_ON, ZV_SEEN, MIRROR_APPLIED, MIRROR_FALLBACK, faults, FW_VER, MIRROR_AGE, FR_CYCLE, DSC_ST_RAW | 100 ms → no torque; if ACTIVE, `stop_now()` (0x92 → 80); auto-P via `gear_step(running=false)`; MIL on |
| 0x309 | PICO_Dsc | 10 ms | — | as draft 1 (FR 0x2B/0x3A requests) | 50 ms → ignore DSC requests, cap torque at the last limit, ramp to normal over 1 s |
| 0x30A | PICO_Brake | 20 ms | — | FR 0x3F pressure, pedal travel, fresh/E2E | 100 ms → regen blending off (keep the 0x173/0x0EF logic) |
| 0x30B | PICO_Cruise | 50 ms | — | FR 0x21 request (log only) | 200 ms → ignore |
| 0x30C | PICO_FrEcho | 100 ms | after every FR 0xE6 it sent | SRC_ALIV, E6_B3/B4/B7 as sent, E6_LAT, RPM4_28 | 300 ms → consistency monitor off |

**Consistency monitor (ZV, optional but cheap):**
- Compare the PICO_FrEcho E6 bytes with what the ZV last put in 0x3F9 d2/d3/d6.
- The two should agree within 1 event. Expected agreement: d2 equal; d3 equal except bits 4–5 while `g_fr_park` ≠ `g_park`; d6 equal.
- Count mismatches lasting longer than 50 ms. This is the in-car check that "both buses agree".

### 6.3 Start-up and fail-safe

* **Link start:** each side starts sending its private frames at W + 20 ms. The Pico mirrors only after it has received 0x304 with MIRROR_EN=1, **and** 0x305, **and** 0x306, all valid, within the last 100 ms. Until then the Pico sends its template (engine off, P), which is exactly v1h.3 with the engine story inactive.
* **ZV lost (Pico side):**
  - ZV_Status or ZV_EngState missing for 100 ms: MIRROR_FALLBACK.
  - If the last mirror had ACTIVE=1 and PHASE CRANK/RUN, the Pico seeds its own `eng_t` (phase RUN, `has_run`, rpm4 = last) and calls `stop_now()`. FlexRay then plays 82 → 92 → run-down → 80/64 ready 1, exactly as v1h.3 would.
  - Gear: `g_set` 0, i.e. P locked.
  - The Pico never restarts the engine story itself in private mode. It goes back to mirroring when the ZV returns with PHASE OFF/STOPPED, never mid-run.
* **Pico lost (ZV side):**
  - FlexRay DDE frames are probably gone as well, and the car will log DSC/EPS faults.
  - ZV: torque 0, engine story `stop_now()`, auto-P, MIL on.
  - Keep the BMW PT-CAN set running: the car still needs them to sleep cleanly.
* **Bus quiet:** both stop at once (200 ms), with the same rule (§3.1).

---

## 7. FlexRay ↔ PT-CAN consistency and the latency budget

### 7.1 What must agree

All of these come from one `eng_out_t`, so they agree by construction if the Pico mirrors it.

| Signal | PT-CAN (ZV) | FlexRay (Pico) | Source field |
|---|---|---|---|
| Engine state 80/81/82/92 | 0x3F9 d2 | 0xE6 b3 | e6_b3 |
| Phase byte | 0x3F9 d3 | 0xE6 b4 | e6_b4 (+ park bits) |
| Start-ready | 0x3F9 d6 hi | 0xE6 b7 hi | e6_ready |
| Crank marker | 0x0D9/0x3F9 ctr_hi 3 | E2E group CounterHi 3 on 0x28/0x29/0x3D/0xE6 | cnt_hi |
| rpm | 0x0A5 d5..6, 0x0F3 | 0x28 c1 b6..7 | rpm4 |
| Torque | 0x0A5 d2..4 | 0x28 c1 b3..5 | torque |
| Gear nibble | 0x3F9 d6 lo | 0xE6 b7 lo | g_nib |
| P lock | 0x3F9 d3 b4-5 = `g_park` | 0xE6 b4 b4-5 = **`g_fr_park`** (deliberately 2 while releasing) | g_park / g_fr_park |
| Gear echo | 0x0DC d2/d5 | 0x28 c3 b11/b14 | g_dc_b2/b5 |
| Sensor-valid / run nibble | 0x08F d2..4 | 0x29 c3 | f97c, r29_b5 |
| FB9 | 0x3B3 d1..3 | 0xFB L9 b2..4 | fb9_* |
| Coolant/oil | 0x3F9 d4/d5 | 0xE6 b5/b6 (new patch) | T3F9_D4/D5 |
| MIL | 0x592 | no FlexRay field identified | — |

### 7.2 Latency of the CAN hop

From the ZV's 5 ms engine step to FlexRay:

| Step | Time |
|---|---|
| ZV step → 0x306 queued | 0–1 ms (Task1Ms) |
| CAN frame on the wire (8 B, ~111–135 bits) + arbitration behind ≤ 1 frame | 0.25–0.55 ms |
| Pico MCP251863 RX → core 0 → mailbox | ≤ 0.5 ms (SPI poll loop) |
| Core 1 picks it up at the next FlexRay cycle step (`engine_cycle`, `emu.c:978`) | 0–5 ms |
| 0xE6 only in **even** cycles (`emu.c:1090–1097`) | 0–5 ms more (event copy in the next even cycle) |
| **Total, 0x3F9 → 0xE6** | **~2–12 ms typical, ≤ 16 ms worst** |

For comparison:
- **Real DDE:** 0x3F9 82 at +1002/+783 ms and 0xE6 82 at +1011/+795 ms, i.e. **0xE6 9–12 ms after 0x3F9** (`dde_start_sequence.md` §2.7). The new split is inside the real spread.
- **v1h.3 today:** both from one snapshot. 0x3F9 within ~1 ms, 0xE6 at the next even cycle (0–10 ms).

**0x28 c1 rpm/torque:** 0x305 every 20 ms, and 0x28 c1 every 20 ms. Up to 20 ms + hop of staleness, which is below the 20 ms period that FlexRay consumers already see. OK.

### 7.3 The accepted timings

* **Start.**
  - v1h.3 run 10: D 199/198 ms, first rpm +3/+13 ms, 82 at +176/+186 ms, CAS releases 24/23 ms after our running report.
  - With the ZV: ST_KL reaches the ZV on 0x12F about 7 ms after FlexRay 0x74, so the start begins about 7 ms later than in v1h.3. Every PT-CAN frame (0x3F9 81/82/ready, 0x0A5 rpm) keeps its exact profile timing relative to that start, because it is local to the ZV. FlexRay 0xE6 follows ≤ 16 ms behind.
  - The CAS only *extends* D while it waits, so a late start costs nothing as long as it is far below the (unmeasured) CAS crank timeout.
  - To win the 7 ms back, feed `PICO_Status.FR_STKL` (event-sent on change) into `eng_input_stkl` as well, as v1h.3 does with both sources (`stmb[0]` FR / `stmb[1]` CAN, `emu.c:247, 992–998, 1462, 1544`).
  - **Low risk.**
* **0xE6 even-cycle rule.** The Pico enforces it as today; mirroring does not touch it. **No risk.**
* **0x92 within ~15 ms.**
  - v1h.3 reaches +13.7 ms from FlexRay 0x74 because it reads ST_KL directly from FlexRay.
  - With the ZV, using 0x12F only: +7 (gateway) + ≤ 5 (step) + t92 2 + ≤ 16 (hop + even cycle), i.e. **~15–30 ms** on FlexRay. 0x3F9 92 itself comes at ~+14 ms after the FlexRay 0x74 change.
  - Using FR_STKL from PICO_Status: about 5 ms less.
  - The real DDE: 0x3F9 92 at +19 ms after ST_KL 8.
  - Run 10 had 92 at +728/+746 ms with no visible effect (a checker FAIL only).
  - **Low risk;** it only exceeds the v1h.3 checker target.
* **Gear.**
  - All PT-CAN gear timings (40/20/10/21/42/881/900/340 ms) are local to the ZV at 1 ms resolution, with no hop. This is better than putting the gear on the Pico and relaying it.
  - FlexRay 0xE6 gear bits follow 2–16 ms later. The real EGS→DDE path also relays through the DDE: 0x39A → DDE → 0x3F9/0xE6.
  - **Low risk.**
* **What only worked because one box sent both buses.**
  - (1) 0xE6 and 0x3F9 could never disagree for longer than one even cycle.
  - (2) A FlexRay stall (500 ms without stepping) automatically dropped the PT-CAN overlay to the templates (`eng_snap_select`).
  - (3) The engine story read ST_KL from whichever bus was earlier.
  - In the split: (1) becomes "≤ 16 ms, monitored by PICO_FrEcho"; (2) becomes the 100 ms link timeouts of §6.3; (3) becomes the optional FR_STKL input.
  - Nothing else in v1h.3 relies on shared state between the buses: PT-CAN templates use W/ON/OFF from 0x12F, FlexRay templates use the FlexRay KL15 timeline (0xF1).

---

## 8. Pico firmware change ("private only" mode), for a later v1i

### 8.1 Mode

Add `PTCAN_MODE_PRIVATE`: the build default for the ZV car, also selectable over USB.

**Behaviour:**
- `pte_set_mask(&core, 0)` permanently: no BMW ID can be scheduled. Keep `pte_id_forbidden()` as the second barrier, and add a **whitelist** in `ptcan_emu_service`/TX: only 0x308–0x30F may go to `mcp251863_send_std`. Anything else → `PTE_FAULT_MODE`, back to listen-only (`ptcan_emu.c:318–324`).
- The controller is in Normal mode only while the private gate is RUN; Listen-only otherwise (`set_mode`, `ptcan_emu.c:~100–112`).
- Keep the `pte_on_rx` feed for wake/KL15/state tracking and logging (`ptcan_emu.c:130–136`).

### 8.2 Gate and RX filters

**Gate:** use the same "alive" rule as the ZV (§3.1): 0x0EF/0x12F/0x173 within 200 ms. Ignore 0x304–0x30F and also the ZV's BMW frames, so the two boxes cannot keep each other awake.

**MCP251863 filters:**
- Keep accept-all on the logging FIFO, because the Pico is also the logger.
- If a dedicated FIFO is wanted for latency, add a filter on 0x304/0x7FC (ZV frames) with its own RX FIFO and interrupt, so ZV_EngState is not behind ~900 fps of car traffic.

### 8.3 Engine/gear source

In private mode:
- `engine_cycle()` (`emu.c:978–1006`) skips `eng_step`/`gear_tick`. Instead it takes the latest consistent `eng_out_t` decoded from 0x305/0x306/0x307 (seqlock as `stmb`, `emu.c:247`) and calls `engine_publish()`.
- The E6 event detection (`emu.c:901–908`) and the even-cycle rule stay as they are.
- New FlexRay patch: E6 b5/b6 = T3F9_D4/D5 when ≠ 0.
- `dde_engine_on_can()` (`emu.c:1541`) still decodes 0x12F/0x173/0x197 for logging only.
- Fallback per §6.3 uses the existing `stop_now()` path. That needs a small `eng_seed_running()` helper.

### 8.4 Foreign / duplicate-ID check

- In private mode, `own_id_enabled()` (`core.c:203–207`) is false for all BMW IDs (mask 0). The ZV's 19 frames therefore no longer fault the Pico.
- **Add** the check for the private Pico IDs instead: any received 0x308–0x30F → `PTE_FAULT_FOREIGN`, stop private TX, report `FLT_FOREIGN_PRIV`.
- **Keep** the legacy check unchanged in legacy mode. If someone flashes a legacy v1h.3 build into the ZV car, the Pico sees the ZV's 0x3F9/0x0A5 and goes silent (FAULT_FOREIGN). The ZV sees the Pico's and goes silent too (§3.6). Fail-silent on both sides, which is the safe outcome.

### 8.5 New TX

- 0x308–0x30C per §6.2 and the DBC.
- 0x309 needs the FlexRay decode of 0x2B/0x3A, 0x30A of 0x3F, 0x30B of 0x21. These are new FlexRay RX decoders, using the layouts in `dde_dsc_torque_rx.md`, `dde_brake_rx.md` and `dde_cruise_rx.md`.

---

## 9. Bus load

**Method:** `frame_bits()` in `/workspace/scratch/pzv/scan.py` gives the exact classic-CAN bits including stuff bits. The private frames are averaged over random payloads; the worst case is 135 bits per frame.

| Item | fps | Load @ 500 kbit/s |
|---|---|---|
| Stock car PT-CAN (no DDE/EGS sender) | ~876–887 | 18.5–18.7 % mean, 21.3 % peak |
| Emulator runs (car + 19 Pico frames) | 891–938 | 18.8–19.7 % mean, 22.2 % peak |
| The 19 BMW frames themselves (Pico TX logged) | 280–320 | 6.2–6.9 % |
| **The same 19 frames, now from the ZV** | 280–320 | same: **net change 0** |
| PR #275's 9-frame subset, which would have *duplicated* the Pico's | 254 | 5.8 % (disappears once ownership is single) |
| ZV → Pico: 0x304 50 + 0x305 50 + 0x306 ~55 + 0x307 ~12 | ~167 | ~3.8 % |
| Pico → ZV: 0x308 50 + 0x309 100 + 0x30A 50 + 0x30B 20 + 0x30C ~15 | ~235 | ~5.4 % |
| **Private link total** | **~402** | **9.2 % typical, 10.9 % worst** |
| **PT-CAN total, new design** | ~1300–1340 | **~28–29 % mean, ~33 % peak** |

This is lower than draft 1 (354 fps but 8.1/9.6 %) plus duplicates. The biggest single item is PICO_Dsc at 10 ms. If load ever matters, 20 ms is acceptable for logging-only use.

---

## 10. Risks

1. **Start versus precharge** (§3.5a). If 82 waits for MOD_RUN, the CAS may give up on a slow precharge; its timeout was never measured. **Mitigation:** profile-driven 82, torque gated separately, 5 s precharge watchdog → 0x92.
2. **0x3F9 park bits in the PR** claim "P secured" in D. The CAS/BDC reflects them into 0x12F d5 bit 2 within 4 ms (run 6). That is a real behavioural bug today: start/stop interlocks and "P" chimes. Fix it before any drive.
3. **ZV power after OFF.** NM to OFF+46 s, 0x3FD P to OFF+30.2 s, 0x592 final at OFF+54 ms, and wakes without KL15 (0x512 ×4, 0x518 for 30 s) all need the ZV powered and awake whenever PT-CAN is awake. If the ZV runs from KL15, expect NM/"ECU missing" entries after every drive. Check the ZV supply and its sleep behaviour.
4. **Scheduling resolution.** Porting the scheduler into `Task10Ms` instead of `Task1Ms` shifts event timing by up to 10 ms. The run-10 gear/CAS timings were tuned to ±2 ms. Use `Task1Ms`.
5. **TX queue / mailbox.** At the 20 ms boundaries the ZV queues 0x0A5, 0x0D9, 0x0DC, 0x0F3 (+ 0x08F, + 0x304–0x306) together with the inverter/BMS frames on the same CAN. `SENDBUFFER_LEN` 20 drops silently when full. Count drops; consider putting the BMW set on its own CAN interface if the inverter shares it.
6. **FlexRay/PT-CAN skew** (§7.2) grows to ≤ 16 ms, inside the real DDE's 9–12 ms typical but not identical. Watch PICO_FrEcho.E6_LAT in the first run.
7. **0x92 timing** grows to ~15–30 ms on FlexRay (§7.3). It was cosmetic in run 10.
8. **Fallbacks are untested.** Neither the ZV-lost nor the Pico-lost path (§6.3) has ever run in the car. Test on the bench by pulling the link while "running".
9. **Port drift.** Re-implementing `build()`/`set_outputs()`/`gear_*` by hand will drift, as the PR already did (§2). Copy the C files verbatim and test with the existing host vectors (`tests/host/ptcan_e2e_vectors_run4.txt`, `gear_vectors_r10md10m.txt`, `run9_ptcan_rx.txt.gz`). Licence: `pico-flexray` has no LICENSE file at the root and Stm32-vcu is GPL. Confirm before copying.
10. **Real pedal on 0x0D9 and real voltage on 0x281** never ran in the car with the DDE/EGS emulation. Keep both behind parameters.
11. **N gear bytes** (0x39A 01, nibble 1) are still a hypothesis (`gear.c:36`). Run 10 drove through N without complaint, but this was not captured on a real 8HP.
12. **Parking.** "P" is display only (§3.7).

---

## 11. Decisions for Damien

| # | Question | Recommendation |
|---|---|---|
| 1 | Does 82 ("running") follow the fake profile or the inverter's Run state? | Profile (SHORT), torque only in Run, 5 s watchdog |
| 2 | Port the v1h.3 C files verbatim, or rewrite? | Verbatim, driven from `Task1Ms` |
| 3 | 0x0A5 torque with the engine stopped: v1h.3 269.5 Nm or the PR's 0 Nm? | v1h.3 (proven); 0 Nm only after a test |
| 4 | Real pedal on 0x0D9 / real 12 V on 0x281? | Parameters, default v1h.3 values (0 % / template) |
| 5 | Feed FR ST_KL from the Pico into the ZV engine story? | Yes (gains ~7 ms, cheap) |
| 6 | ZV powered ≥ 50 s after OFF and on wakes without KL15? | Must be yes; check the wiring |

---

## 12. Worked CRC examples (private frames)

- **ZV_EngState (0x306)**, alive 3, engine running in D (g_set 1, f97c 1, e89 1; e6 82/26, ready 0, nibble 5; 0x0DC 24/F4; fr_park 0; phase RUN; gear D):
  `F5 73 82 26 50 24 F4 68`. The Pico puts 0xE6 b3 82, b4 (26 & CF) | 0 = **06**, b7 hi 0, lo 5.
- **ZV_EngFast (0x305)**, alive 5, active, 780 rpm (rpm4 0x0C30), torque 2098, c3_b34 1, c3_b56 2, turbine 2724:
  `A1 15 30 0C 32 98 A4 0A`.
- Both decode correctly with cantools against the DBC.
