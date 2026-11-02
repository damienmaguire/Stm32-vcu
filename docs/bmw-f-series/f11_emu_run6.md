# F11 emulator run 6: full DDE + EGS emulation (DDE 12 V off, EGS fuse pulled), v1e, PT-CAN TX on

Analysis 2026-10-31 (Europe/Dublin). Baseline: run 4 `ddecan` (real DDE + EGS). Secondary: run 5 `noegs`.
Damien's report: dash clean, MIL went out. **Verdict: the capture agrees. The PT-CAN + FlexRay emulation is clean. The bus slept on time, and no new check-control or diagnostic traffic appeared.** The issues left are small timing and fidelity items (see §8).

## 0. Files and time base
- The archive held only `f11_emu6_fullemu.csv.bin` (30.7 MB). It was rebuilt with `fr_ts_logger.py --replay … --can` (pico-flexray-emu `dde-emulator` branch) into `/workspace/f11-emu-run6/replay/`. Replay totals equal the console `done:` line: 752,798 FR frames, hw_ts 99.3 %, seq_gaps 0, 79,733 CAN frames, 107 IDs, bytes_skipped 0.
- t = 0 at the first FR frame. Console `[N s]` ≈ t + 6.8 s.
- **The capture does not contain our own PT-CAN TX.** The MCP251863 does not record its own frames, so none of the 19 emulated IDs appear in the CSV. The TX stream was therefore reconstructed: the run 6 RX traffic was replayed through the v1e core (`tests/host/test_ptcan_emu.c replay`, mask 0x7FFFF), giving `replay/run6_sim_tx.txt`.
  - The simulated per-ID counts equal the firmware's console counters exactly for every ID except the 20/30 ms ones and 0x3FB. Those exceptions are listed in §2.
  - Content, CRC and timing below are from that reconstruction. Bus-side effects (0x12F, NM, CC, sleep) are measured directly from the capture.

## 1. TX set and CAN TX health (console)
- argv `--emulate --can --can-tx`. Mask = all 19 IDs (3F9 3FD 39A 396 518 0A5 0D9 0DC 0F3 08F 2C4 281 1B9 381 3B3 397 3FB 512 592).
- `ctl=Normal` and state RUN from wake until the bus went quiet, then `silent` → `wait-bus`. ListenOnly appears only at Ctrl-C (`PT-CAN TX DISABLED`).
- There are no `[CANTX] FAULT` lines. Foreign-ID collisions: none, and the capture has 0 frames from any other sender on the 19 IDs. fifo-full 0, fail 0, REC 0. **TEC touched 4 once** (around console 15 s, a few seconds after KL15 on), then went back to 0. That is one transient error frame.
- tx total 35,361.

## 2. Emulated IDs vs real DDE/EGS (`f11_emu_run6_tx_vs_ddecan.csv`)
- **Content:** all 19 IDs have the same DLC and the same modal KL15 and after-off payloads as `ddecan`.
- **E2E:** the 7 E2E IDs (3F9, 3FD, A5, D9, DC, F3, 8F) pass CRC8 0x1D with their data IDs at 100 %, with 0 counter breaks.
- **Start after wake:** same as the baseline.
- **Stop after OFF:**
  - Cyclics: OFF+51.9 to +52.6 s (real +52.5 to +52.6).
  - 0x512: +45.67 (real +45.93).
  - 0x518: +29.67 (real +29.91).
- **Periods match** (20/30/100/645/1000/5000/10000 ms), with these exceptions:
  - **Real TX rate of the fast frames is about 2.5 % low.**
    - 0A5/0D9/0DC/0F3: firmware sent 6,447 against 6,634 scheduled. That is 244 per 5 s console interval, about 48.6/s (≈20.6 ms).
    - 08F: 4,298 vs 4,423.
    - 2C4/281: 1,308 vs 1,326.
    - 3FB: 223 vs 225.
    - 100 ms 0x3FD lost nothing.
    - Likely cause: the poll loop sends at most 4 frames per call, and a cyclic slot more than one period late is dropped rather than caught up. No reaction was seen.
  - 0x397: 1000 ms constant `c0 e2 00`. The real one was ~380 ms, with `e3` blips and a `df/de/dd` d1 walk after OFF. Not reproduced.
  - 0x592: only the MIL-off CC frame (`40 22 00 14`, CC 34 cleared), about every 5 s during KL15 (16 frames) and once at OFF+0.05.
    - The real DDE sent CC 34 **active** (`…15`), which is why the MIL went out.
    - The real DDE also sends undecoded service frames (`26…` at OFF+0.22 and then every 5 s until +50 s, `30…`/`32…` at OFF+0.4/0.5). These are not emulated.
- **0x3FD vs Ian's `egs_gws_lever_can.md`:** DLC 5, 100 ms, data ID 0x39 (1327/1327 CRC OK), B1 high nibble 0, counter 0..14.
  - B2 = 0x00 before KL15, then 0x20 (P) from ON+0.04 s to OFF+30.24 s (real +30.16), then 0x00.
  - B3/B4 = `FC FF`.
  - **This matches the real EGS.**
- **0x197 (GWS lever) is present:** 4,351 frames, DLC 4, data ID 0x3F, CRC 4351/4351, median 30 ms (mean 30.5 ms).
  - B1 high nibble 0, **B2 = 0x0F, B3 = 0xC0** constant (rest position, P button not pressed). Identical to `ddecan`.

## 3. 0x12F
- ST_KL 6→9→**A at 4.807 s (ON)**. OFF: **A→7 (80.507) → 6 (80.607)**. **Final value 6** (last frame d2 low nibble 6), not 8. ✔
- **d5 bit2 (field42):** clear at wake. It sets at 0.597 s, 4 ms after the emulated 0x3F9 reaches b28-29 = 2. It then stays set through KL15 and the OFF press until sleep. ✔ (ddecan: set at wake, dip 0.527–0.602.)
- Small difference at the OFF press:
  - ddecan d5 goes 85→**45**→05. Run 6 goes 85→05.
  - Run 6 instead shows **d4 low nibble = 4 from OFF−0.23 s to OFF+9.8 s**. This also appeared in noddecan/noegs but never at OFF in ddecan. All runs show it for 10 s at ON.
  - It starts before OFF, so it is the master's reaction to the button press, not to our post-off frames. No visible consequence. Cause unknown.

## 4. Bus sleep and NM
- Last CAN 133.180 s = **OFF+52.67**. Last FR 133.280 = **OFF+52.77** (baseline +52.65 / **+52.75**). ✔
- **0x501 and 0x560 released**: last frames OFF−0.25 and OFF−0.54 s (in noegs they never released).
- 0x510 +50.66 (baseline +50.63), 0x55E −0.35, emulator FR 0x5C +47.02 (baseline +46.99).
- 0x52A (node 0x2A) ran to OFF+3.82 (baseline −0.19). Harmless.

## 5. New or unexpected PT-CAN IDs vs ddecan (`f11_emu_run6_ptcan_diff.csv`)
- **No new IDs.** Vanished: the 19 emulated IDs (not captured, see §0), 0x351 (DDE event, excluded), and 0x62A/0x65E/0x6F0 (tester traffic only in ddecan).
- **No 0x6xx diagnostics.**
- **Check-control frames** (`0x580+node`, `40 lo hi st`): the set is the same as ddecan. ACSM 77/181, DSC cleared set, 0x5B8 45/245 cleared, 0x5C0 148/771, lights 120/256/378.
  - There is no EHC 245 active and no 0x5C0 14/18. Both were present in noegs.
- 0x27A (one frame at OFF+32.5) is back, as in ddecan (+32.25). It was absent in noegs, so it is probably triggered by the 0x518/0x3FD stop.
- Other diffs are ambient (temperatures, time/odometer, 0x163/0x2A5/0x32E …).
- **FR side confirms the gateway accepts our frames:**
  - FR 0xE9 now carries `01 10 f1 58`, the emulated 0x3FB tail.
  - FR 0x74 (0x12F copy) b6 01→05 = field42.

## 6. Immobiliser
- The ZV never asks the car for an immobiliser exchange, and in testing the car never asked for one, so nothing is needed.
- 0x12F reached A normally, and field42 is set. A crank/READY request was not exercised (there is no engine), so START-path ST_KL values (B/C) were not tested.

## 7. FlexRay (console + `nod_fr.py fullemu`, `f11_emu_run6_fr_diff.csv` vs noegs)
- Emulator ACTIVE/locked from 10.1 s console until the bus went to sleep.
  - echo **miss 0, foreign 0**, late 28 (all `win`).
  - lock_losses 1 (= bus sleep), rearms 1 (start-up).
  - skip E6=1, E8L9=2; skip2 0x142=26, 0x68=1.
- Statics 0x26/0x28/0x29/0x3D: E2E 100 % valid. E6 CRC 133/133. Gaps only at the 27 logger-poll holes.
- 0x142 shows 51 gaps >30 ms (noegs 35). This matches skip2 142=26, which is new but harmless.
- E8 L5 and 0x10A are KL15-gated, as before. DSC ready 0x2F b4 0e→02 at ON+1.04.
- **FB L9 after OFF is now `1f 00 ff 7f 00 f3 ff ff`** (from OFF+5.07 s, the first 5 s slot). It was `…04 00 00…` in v1d/run 5, and the new value matches the real DDE's 0x3B3 post-off `1f 00 ff 7f`. ✔ 0x29 b8 also changes after OFF (10→c0 vs noegs).
- FR diff vs noegs:
  - No slot new or vanished in the KL15/off phases, except 0xFD/0x103/0x10C, which stop after OFF because the bus now sleeps.
  - Content diffs are run-to-run values plus the gateway copies above.

## 8. Remaining mismatches / v1f suggestions
1. **Fast-frame drop (~2.5 %)** on 20/30 ms IDs:
   - Raise the per-poll TX budget (4 → 8+).
   - Catch up late slots instead of skipping a period (or keep the grid with a 1-period tolerance).
   - Find what stalls the loop more than 20 ms (look at the 5 s stats/print path).
2. **Record own TX:** use the MCP251863 TEF (TX event FIFO with timestamps) into the capture stream. Then TX content and timing can be verified from the capture instead of a model.
3. 0x397: add the `e3` blips and the post-OFF d1 walk (df/de/dd), or at least the ~380 ms cadence. Optional.
4. 0x592: real DDE also sends `26/30/32` service frames after OFF (+0.22/+0.42/+0.52, then `26` every 5 s to +50 s). They are undecoded, so **do not emulate blindly**. Keep watching whether anything misses them.
5. 0x12F OFF path: d4 = 4 for 10 s / no 0x45 step. Cause unknown. If it matters, compare the DDE frames in the 1 s before the press (the real 0xA5 torque flip 1aaa/199a is not reproduced).
6. Minor: 0x512/0x518 stop ~0.25 s earlier than real, and the 0x3FD P-hold ends 80 ms late (100 ms grid). The FB L9 post-off switch waits for the next 5 s slot (OFF+5.07).

Scripts: `analysis/emu_run6_scripts/` (`run6.py`, `simcmp.py`, `nmcc.py`, `misc.py`, `diff_ids.py`, `fr_diff.py`, `nod_fr.py`, `load4.py` with `fullemu`).
