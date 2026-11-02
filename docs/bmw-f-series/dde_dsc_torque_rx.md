# DDE ← DSC torque / intervention RX (EDC17C41 / TC1797)

Documentation only, owned hardware. Labels: **[CONFIRMED]** / **[ANALYSIS]** / **[HYPOTHESIS]**.
Flash addresses cited throughout.
Date: 2026-10-31 (Europe/Dublin).

Cross-refs: `dde_flash_pdu_layouts.md` §5, `dde_flash_schedule.csv`, `dde_flash_mb_table.csv`,
`dde_flash_signals.md`, `dde_flash_e2e.md`, `dde_flash_eray_config.md`, `dde_flash_ptcan_rx.csv`,
`f11_emu2_dsc.md`, `f11_emu3_dsc.md`, companion CSV `dde_dsc_torque_rx.csv`.

---

## 0. Scope and sender split

The DDE receives **no DSC IDs on PT-CAN**. PT-CAN RX torque requests are EGS-only
(`0xB0` / `0xA0`, 20 ms, 12-bit 0.5 Nm −1023.5, substitute 32767) — see
`dde_flash_ptcan_rx.csv`. All DSC ↔ DDE torque / stabilisation traffic is **FlexRay**.

Public DME DTC clusters separate senders **[ANALYSIS]**:

| Cluster (slot.base.rep) | Public failure context | Assigned sender |
|---|---|---|
| 43.1.4, 43.3.4, 46.1.2, 47.1.2, 263.1.4 | DSC dead (Carly F01 / motor-talk) | **DSC** |
| 33.1.4, 58.1.4, 55.x, 56.x, 57.1.2, 272.4.8 | ICM cluster missing | **ICM** (not DSC) |

`mitm_id_ownership.csv`: 0x21, 0x2B, 0x2E, 0x2F, 0x37, 0x38, 0x39, 0x3A, 0x3F, 0x31 are
vehicle-side (bus0) every cycle.

---

## 1. DSC → DDE FlexRay IDs that carry torque / intervention

### 1.1 Summary table

| FID | base/rep | Public name (DTC text) | PDU | len | E2E data ID | Period ticks | Torque relevance | EV |
|---|---|---|---|---|---|---|---|---|
| **0x2B** | 1/4 (43.1.4) | Anforderung Radmoment Antriebstrang Summe Stabilisierung | **4** | 8 | 0x42 | 2 | **Primary wheel-torque stab request (ASR/MSR)** | **KEEP** |
| 0x2B | 1/4 | Soll Verteilung Längsmoment VA/HA | **5** | 6 (E2E len 5) | 0x5F | 2 | Longitudinal moment VA/HA distribution | KEEP |
| **0x2B** | 3/4 (43.3.4) | Ist Bremsmoment Summe | **20** | 8 | 0x02 | 1 | Actual brake torque sum (2×) | KEEP |
| **0x2F** | 1/2 (47.1.2) | Status Stabilisierung DSC (ST_STAB_DSC) | **12** | 8 | 0x47 | 1 | DSC stabilisation status / pass-check | **KEEP** |
| 0x2E | 1/2 (46.1.2) | Ist Drehzahl Rad | **6** | 12 | 0x11 | 1 | Four wheel speeds | KEEP (context) |
| 0x107 | 1/4 (263.1.4) | Status Fahrzeugstillstand | **34** | 4 | 0x83 | 1 | Standstill status | KEEP (context) |
| **0x3A** | 1/4 (58.1.4) | Anforderung Drehmoment Kurbelwelle Fahrdynamik | **19** | 10 | 0x44 | 1 | **Three crank-torque requests (fast path)** | **KEEP** [ANALYSIS: ICM DTC cluster; still the crank intervention path the DDE arbitrates] |
| 0x21 | 1/4 (33.1.4) | Anforderung Radmoment Antriebstrang Summe FAS | **2** | 6 | 0x03 | 2 | FAS/ADAS wheel-torque request — **not DSC** | optional |
| 0x13 | 3/4 | (unnamed sibling of PDU5) | **13** | 6 | 0x5E | 2 | Sibling long-moment field | low |

PDU 21 shares 0x2B 3/4 but has `variant_mask 0x0000` (disabled) and 0 signals **[CONFIRMED]**.

MB table flash anchors: 0x2B 3/4 @ `0x80231B8C` (entry 5); 0x2B 1/4 @ `0x80231BCC` (entry 9);
0x3A @ `0x80231BDC`; 0x21 @ `0x80231BEC`; 0x2F @ `0x80231C0C`; 0x2E entry 15; 0x107 @ `0x80231D8C`.

COM PDU descriptor base: `0x8023BF84` (60 × 0x30). Signal list @ desc+8; E2E cfg @ desc+0x28.

---

### 1.2 Bit layouts (Intel / LSB-first)

All signals use packer/unpacker callback `cb=0x80124D8E`. Overflow/flag ptr `d0006C1A`.
Validity id **658 = always valid** on these RX descriptors (gate is E2E + qualifier, not vid).

Scaling convention used below matches prior RX decode:
`raw = (phys − p1)·p2 / (p0·p3)`; wheel-torque family verified as **1 Nm/bit, offset −32000**
(idle raw 32000 → 0 Nm) **[CONFIRMED]** against run4.

#### PDU 4 — 0x2B base1 offset 0 (8 B) — E2E ID 0x42 — **[CONFIRMED]** flash `0x8023C044`

| start bit | len | type | RAM | SNA / subst | meaning |
|---|---|---|---|---|---|
| 0 | 8 | crc8_e2e | d0006F8B | — | CRC8 poly 0x1D + data ID |
| 8 | 4 | alive_0_14 | d00059FA | — | alive 0..14 |
| 12 | 4 | u8 | **d0007031** | 15 | Qualifier |
| 32 | 12 | u16 | **d0005ACA** | 4095 | Companion 12-bit (idle raw 0xE00) |
| 44 | 4 | u8 | **d0007062** | 15 | Qualifier 2 |
| 48 | 16 | u16 | **d00030E8** | 0xFFFF | **Wheel-torque stab request**, 1 Nm −32000 |

E2E cfg @ `0x80241570`: callback `0x80124E12`, fault-index `0x80229412` → **0xFD**,
cal `0x8034D47A`, data ID 0x42, len 8.

#### PDU 5 — 0x2B base1 offset 8 (6 B, E2E len 5) — E2E ID 0x5F — **[CONFIRMED]**

| start | len | type | RAM | SNA | meaning |
|---|---|---|---|---|---|
| 0 | 8 | crc8 | d0006F65 | — | |
| 8 | 4 | alive | d00059C0 | — | |
| 16 | 16 | s16 | **d0004D66** | 0xFFFF | Long-moment VA/HA, 1 Nm −2500 (idle raw 2500 → 0 Nm) |
| 32 | 8 | u8 | d00070DF | 255 | Qualifier / mode |

E2E cfg callback `0x80124F26`, fault-index → **0x6C**, cal `0x80373C1E`.

#### PDU 20 — 0x2B base3 offset 0 (8 B) — E2E ID 0x02 — **[CONFIRMED]**

| start | len | type | RAM | SNA | meaning |
|---|---|---|---|---|---|
| 0 | 8 | crc8 | d0006F5B | — | |
| 8 | 4 | alive | d00059AE | — | |
| 12 | 4 | u8 | d000702C | 15 | Qualifier |
| 16 | 16 | u16 | **d00030CC** | 0xFFFF | Brake-torque A, 1 Nm −32000 (idle 0 Nm) |
| 36 | 4 | u8 | d0006FEB | 15 | Qualifier 2 |
| 40 | 16 | u16 | **d00030C8** | 0xFFFF | Brake-torque B, 1 Nm −32000 (idle 0 Nm) |

E2E cfg callback `0x80124E74`, fault-index → **0x4F**, cal `0x8034D470`.
Capture note: emu fail pattern flips 0x2B c3 b11/b12 `e0/f8` → `60/d8` when DDE missing
**[CONFIRMED]** `f11_emu2_dsc.md` — those bytes sit in this PDU's tail / pad region.

#### PDU 12 — 0x2F base1 (8 B) — E2E ID 0x47 — ST_STAB_DSC — **[CONFIRMED]**

| start | len | type | RAM | SNA | meaning |
|---|---|---|---|---|---|
| 0 | 8 | crc8 | d0006F78 | — | |
| 8 | 4 | alive | d00059D2 | — | |
| 12 | 4 | u8 | d000702F | 15 | Qualifier |
| 28 | 12 | u16 | **d0005A48** | 4095 | Status field A (idle 0xE00) |
| 40 | 12 | u16 | **d0005A4C** | 4095 | Status field B (idle 0xE00) |
| 56 | 6 | u8 | d0006FE9 | 63 | Status nibble (idle 0x20) |

E2E cfg callback `0x80124E80`, fault-index → **0xBE**.
**Pass-check bytes (capture):** at ign+≈1.05 s with real DDE, 0x2F c1 b4 `0e→02`, b5 `30→20`;
DDE-missing stays `06/30` and b7→`f6` **[CONFIRMED]** `f11_emu3_dsc.md`.

#### PDU 19 — 0x3A base1 (10 B) — E2E ID 0x44 — crank Fahrdynamik — **[CONFIRMED]** layout / **[ANALYSIS]** sender

| start | len | type | RAM | SNA | meaning |
|---|---|---|---|---|---|
| 0 | 8 | crc8 | d0006F85 | — | |
| 8 | 4 | alive | d00059EC | — | |
| 12 | 4 | u8 | **d0007022** | 15 | Qualifier (idle = SNA 15) |
| 16 | 12 | u16 | **d0004D7C** | 4095 | Crank request A, **0.5 Nm/bit, no offset** |
| 28 | 12 | u16 | **d0004D76** | 4095 | Crank request B, 0.5 Nm/bit |
| 52 | 12 | u16 | **d0004D7A** | 4095 | Crank request C, 0.5 Nm/bit |
| 64 | 8 | u8 | d0007021 | 255 | Mode / flags (idle 0x82) |

E2E cfg callback `0x80124E20`, fault-index → **0xE1**, cal `0x80373C2A`.
Idle run4: all three crank raw = 0 → 0 Nm; qualifier = 15 (SNA).
**[HYPOTHESIS]** the A/B/C triple is ASR-fast / ASR-slow / MSR (or similar); exact role split open.

#### PDU 6 — 0x2E (12 B) — wheel speeds — **[CONFIRMED]**

Four s16 @ bits 16/32/48/64 → d0004CD6 / 4CDA / 4CCE / 4CD2,
scale `raw=(val+9778)·10000/2984`; four u4 qualifiers @ bits 80–92.
E2E ID 0x11, callback `0x80124DFA`, fault-index → 0x96.

#### PDU 34 — 0x107 (4 B) — standstill — **[CONFIRMED]**

u8 @ bit 16 → **d000766F**, SNA 255. E2E ID 0x83, callback `0x80124E9A`, fault-index → 0xC6.
Idle run4: payload `xx f1 00 ff` / later `xx fN 03 ff` (standstill byte changes with motion).

#### PDU 2 — 0x21 (6 B) — FAS wheel torque — **[CONFIRMED]** not DSC

u4 qual d00070E4; 16-bit **d00030F4** (1 Nm −32000); 12-bit d0005ACE.
E2E ID 0x03. Listed for completeness; ICM/FAS DTC cluster.

---

### 1.3 Period / timeout / substitute

Schedule (`dde_flash_schedule.csv`, flash `0x8023B9D4 + 0x18·i`):

| PDU | period_ticks | initial_phase_ticks |
|---|---|---|
| 4, 5, 2, 13 | 2 | 1 |
| 20, 12, 34 | 1 | 1 |
| 19, 6 | 1 | 2 |

FlexRay cycle ≈ 5 ms on this car **[ANALYSIS]** (`dde_flash_eray_config.md` macrotick math).
So period 1 ≈ 5 ms, period 2 ≈ 10 ms — matches slot repetition (rep 2 → 10 ms, rep 4 → 20 ms).

RX path **[CONFIRMED]**:
- Unpacker: `FUN_801906B6` @ `0x801906B6` (SNA on out-of-range: u16→0xFFFF / 0xFFFE, etc.).
- Monitor: `FUN_80191D88` @ `0x80191D88` — per-PDU deadline counter `d0014839[idx]`.
  Reloaded from schedule period byte (`+0x14`); on zero → state=2 (timeout), then
  `FUN_80190494` / `FUN_80125E7E` fault path.
- E2E-enable callbacks (PDU4 `FUN_80124E12`, PDU19 `FUN_80124E20`) only test a cal flag
  (`*(*(a9+off)+n) != 0`); they do **not** themselves substitute.
- Timeout / E2E-fail substitute for the stab cluster: `FUN_801229DC` @ `0x801229DC`
  (called via `FUN_80122F7C`) writes defaults including
  d0005ACA/5A48/5A4C ← `0x800`, d0007062 ← gate byte, etc. **[ANALYSIS]** exact trigger
  condition = E2E/timeout fail path for PDUs 4/12.

**Open:** absolute ms timeout = `period_ticks × RX_task_tick`; RX task tick not pinned
(same open item as `dde_flash_pdu_layouts.md` §5). Report as period ticks + schedule
**[ANALYSIS]**.

---

## 2. How the DDE applies them (call chain)

### 2.1 Shadow copy layer

`FUN_80160CF0` @ `0x80160CF0` (fptr from `0x80258304`) copies RX RAM → working shadows, e.g.:

| RX | shadow |
|---|---|
| d00030E8 (PDU4 wheel tq) | **d0003684** |
| d0004D66 / d0004D64 | d0005D78 / d0005D76 |
| d0007031 / d0007062 / d0005ACA | d0007637 / d0007643 / d0006054 |

`FUN_80169ADE` @ `0x80169ADE` (called from task @ `0x800DD1CC`) shadows PDU19:

| RX | shadow |
|---|---|
| d0004D76 | **d0005D7A** |
| d0004D7C | **d0004D72** |
| d0004D7A | **d0004D78** |
| d0007022 / d0007021 | d000701F / d0007634 |

`FUN_80185454` @ `0x80185454` (same task) then:
- d0003684 → **d00030E4** (second buffer of wheel-tq request)
- d00030C8 / d00030CC → d0003670 / d0003674 (brake working copies)
- raises faults via `FUN_800E2B60` when monitor flags set

### 2.2 Crank-torque arbitrator (PDU19) — **[CONFIRMED]** consumer

`FUN_80087E90` @ `0x80087E90`:
- Reads shadows **d0005D7A** (← crank B / d0004D76) and **d0004D72** (← crank A / d0004D7C).
- Difference + rate-limit via `func_0xc0006608(..., gain 0x2000)`.
- Clamps to cal min/max at `*(a9+0xBC0)+0x11E/0x120`.
- Writes result to **d000574E** and **d0005FC8** (`a0-0x3AAA`, `a0-0x3230`).

This is the path that turns the three 0.5 Nm crank requests into an internal torque limit
the rest of the coordinator consumes. **[ANALYSIS]** d000574E/d0005FC8 feed the global
min/max torque selector (exact downstream not fully chased this turn).

### 2.3 Wheel-torque / brake consumers — **[ANALYSIS]** partial

- `ld.w` of d00030E4 @ `0x80185228` (in `FUN_801850C8` / `FUN_80185200` region) — uses the
  buffered stab request; exact arbitration into crank min/max still open.
- Brake working copies d0003670/3674 loaded @ `0x80164D76` / `0x80164D7E` and passed by
  pointer from several low-level sites (`0x80018258` …).
- Qualifier / status bundle also touched @ `0x801229F6` (init/substitute neighbour).

### 2.4 Requested limit vs actual cut

| Stage | What | Evidence |
|---|---|---|
| Raw request | PDU4 d00030E8 wheel Nm; PDU19 three× 0.5 Nm crank | flash descriptors |
| After E2E/timeout | SNA or `FUN_801229DC` defaults | RX path |
| After shadow | d0003684 / d0005D7A / d0004D72 | copy funcs above |
| Arbitrated limit | d000574E / d0005FC8 (crank path) | `FUN_80087E90` |
| Actual cut | not decoded this turn — needs chasing who reads d000574E into injector/fuel/EGS limit | **open** |

EGS PT-CAN `0xB0`/`0xA0` remain a **separate** torque-request channel (gearbox), not DSC.

---

## 3. What the EV emulator / Pico must keep

### Must receive (or pass through from real DSC)

1. **0x2B base1 / PDU4** — wheel-torque stabilisation request (`d00030E8`). This is the
   ASR/DSC drive-torque cut the ZombieVerter must honour.
2. **0x3A / PDU19** — three crank-torque requests. Fast intervention path; even if the
   public DTC cluster names ICM, the DDE arbitrates it into the torque limit
   (`FUN_80087E90`). Pico should pass it through unchanged.
3. **0x2F / PDU12** — ST_STAB_DSC. Required for DSC state / the known pass-check
   (b4/b5 pattern). Without a plausible ST_STAB the rest of the car flags DDE/DSC mismatch.
4. **0x2B base3 / PDU20** — actual brake torque. Lets the coordinator account for brake
   contribution; also carries the b11/b12 qualifier pattern the emu docs watch.

### Should keep (context, low cost)

5. **0x2B PDU5** — VA/HA long-moment distribution.
6. **0x2E** — wheel speeds (DSC).
7. **0x107** — standstill.

### Optional / ignore for torque reduction

8. **0x21** — FAS wheel-torque (not DSC). Only needed if ADAS torque intervention is in scope.
9. ICM sensor frames **0x37 / 0x38 / 0x39 / 0x110** — dynamics / switch, not torque requests.
10. EGS **0xA0 / 0xB0** on PT-CAN — keep only if the EGS stays in the EV build.

### Idle / no-intervention payloads (run4, real DDE, ~t=17 s) **[CONFIRMED]**

| Frame | Idle payload (hex, useful bytes) | Decoded |
|---|---|---|
| 0x2B c%4=1 | `1f f1 00 7d 00 ee 00 7d \| 3b f1 c4 09 70` | wheel tq=0 Nm, qual=15; long-mom=0 Nm |
| 0x2B c%4=3 | `5b f1 00 7d 88 00 7d ff … e0 f8` | brake tq A/B = 0 Nm; b11/b12 = e0/f8 |
| 0x3A c%4=1 | `59 f1 00 00 00 ff 07 00 82` | crank A/B/C = 0 Nm, qual=15, flags=0x82 |
| 0x2F c%2=1 | `e8 f1 00 0e e0 00 fe e0` (pre-pass) / post-pass b4=02 b5=20 | ST_STAB |
| 0x21 | early `ff…`; later `8b 04 00 00 00 fe` | FAS inactive / SNA-ish |

Emulator synthesis for “no cut”: keep alive+CRC honest, wheel/crank requests at 0 Nm,
qualifiers at the idle values above (often SNA=15), and let the real DSC keep producing
0x2F / 0x2B when the Pico is only bridging.

When the DSC **does** intervene, do **not** synthesise — pass the live DSC frames through
to the torque controller (ZombieVerter / Pico) and apply `d00030E8` (wheel) and/or the
PDU19 crank triple as the torque ceiling.

---

## 4. Cross-check vs `f11_emu*_dsc.md`

| Observation in emu docs | This RX decode |
|---|---|
| 0x2F b4 `0e→02`, b5 `30→20` at ign+1.05 s = DSC pass with real DDE | PDU12 ST_STAB_DSC; bytes sit in the status/qualifier region |
| DDE-missing: 0x2F `06/30`, b7 `f6` | Same PDU; DSC degrades when it loses DDE TX, not when RX fails |
| 0x2B c3 b11/b12 `e0/f8` vs missing `60/d8` | PDU20 (0x2B base3) tail — brake-torque frame |
| 0x3F c1 b4 `f3` vs `f6` | 0x3F is ICM-side (not in DSC torque set); listed only as companion indicator |
| emu3 (v1c) PASS, zero DTCs; **no capture includes a DSC intervention or driving with the emulator** | Torque-cut behaviour under real ASR/DSC intervention is **unverified on the bench** — open loop |
| EGS 0xA0/0xB0 called out as DDE RX | Confirmed PT-CAN only; not FlexRay DSC |

---

## 5. Open loops

1. **Downstream of d000574E / d0005FC8 / d00030E4** — how the arbitrated limit becomes the
   actual fuel/injector or motor-torque ceiling (function names past `FUN_80087E90`).
2. **RX task tick** → absolute timeout ms for FlexRay PDUs (period ticks known; tick open).
3. **Qualifier semantics** — idle often shows qual=15 (SNA). Whether “request active” is
   qual≠15, a bit inside d0005ACA/5A48, or only non-zero torque, needs a capture with a
   real DSC intervention (ABS/ASR event).
4. **PDU19 sender** — DTC cluster says ICM; DDE still consumes it as Fahrdynamik crank
   request. Confirm on the wire which node TX 0x3A (ICM vs DSC vs both via gateway).
5. **A/B/C role split** for the three crank fields (ASR fast/slow/MSR hypothesis).
6. **No intervention capture** — emu3 never drove with DSC cutting; need a log with
   ASR/MSR active to validate scaling under non-zero requests.
7. E2E-cfg cal pointers (`0x8034D47A`, `0x80373C2A`, …) — debounce / enable thresholds not read.

---

## 6. Paragraph for Sam

The DDE does not take DSC torque cuts on CAN — only on FlexRay. The messages that matter
for EV torque reduction are: **0x2B base1** (slot 43.1.4) carrying the stabilisation
wheel-torque request (1 Nm/bit, offset −32000, RAM d00030E8), **0x3A** (58.1.4) with three
0.5 Nm crankshaft requests the DDE rate-limits into an internal limit (FUN_80087E90),
**0x2F** ST_STAB_DSC for intervention/pass state, and **0x2B base3** actual brake torque.
Idle is “0 Nm + SNA qualifiers”; the Pico should pass the live DSC frames through and have
ZombieVerter honour the wheel/crank requests as a torque ceiling. EGS 0xA0/0xB0 on PT-CAN
are a separate gearbox channel. We still need a real ASR/DSC-intervention capture to pin
qualifier semantics and the final cut path past the arbitrator.
