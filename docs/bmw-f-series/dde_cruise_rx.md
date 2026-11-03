# DDE cruise / ACC / GRA / Tempomat RX–TX map (EDC17C41 / TC1797)

Documentation only — owned F11 520d hardware.
Labels: **[CONFIRMED]** / **[ANALYSIS]** / **[HYPOTHESIS]**. Captures without cruise engagement are marked **NEEDS CAPTURE**.
Date: 2026-10-31 (Europe/Dublin).

Cross-refs: `dde_dsc_torque_rx.md` (FR 0x21 PDU2), `dde_brake_rx.md`, `dde_flash_pdu_layouts.md` §5,
`dde_flash_tx_rx_signals.csv`, `dde_flash_ptcan_tx.csv`, `dde_ptcan_emulation_plan.md`,
`egs_8hp_map.md` / `egs_8hp_ids.csv`, A2L `/workspace/8hp-firmware/ecu.a2l` (GA8P75HZ / SH725xx).
Companion CSV: `dde_cruise_rx.csv`.

---

## 0. Bottom line

| Question | Answer | Label |
|---|---|---|
| Does the DDE decode a cruise stalk / Set / Resume / Cancel / +/- / distance? | **No.** No GRA/Tempomat/Cruise/ACC/FGR/DCC strings in flash; no stalk PDU in the static/dynamic RX set. | CONFIRMED |
| What does the DDE get for cruise/ACC? | **FR 0x21 PDU2** — public name *Anforderung Radmoment Antriebstrang Summe FAS* (ICM/LDM → DDE wheel-torque request). | CONFIRMED |
| Set-speed / button / KOMBI cruise lamp? | Live in the **ICM** (and KOMBI via ICM). DDE does not own them. | HYPOTHESIS (public F-series) + ANALYSIS (no DDE RX) |
| DDE → body cruise status? | FAS arbiter status enum **`d00075DB`** is packed into **PT-CAN 0xD9 bit56 (B7 low nibble)** and mirrored as **`d0006E5F` on FR 0x28 PDU52 bit56**. | CONFIRMED (packing); ANALYSIS (enum meaning) |
| Cruise engagement in owned logs? | **None.** FR 0x21 qualifier stayed 0 / SNA; 0xD9 status was 0 (idle) or briefly 1 with pedal pressed. | NEEDS CAPTURE |

**EV / ZV:** honour FR 0x21 as the cruise/ACC torque ceiling when the qualifier is active; keep publishing 0xD9 / FR 0x28 (pedal + FAS status) and the other MUST TX set so the ICM can run LDM and detect pedal override. Do not invent CAS/BDC body frames. Do not invent stalk or set-speed frames.

---

## 1. Cruise RX — FlexRay 0x21 PDU2 (FAS wheel-torque request)

### 1.1 Frame

| Field | Value | Label |
|---|---|---|
| FID | **0x21** base1 / rep4 (public slot 33.1.4) | CONFIRMED |
| PDU index | **2** (desc `0x8023BFE4`) | CONFIRMED |
| Len | 6 | CONFIRMED |
| E2E | CRC8 poly 0x1D, data ID **0x03**, cfg `0x802415B8`, callback `0x80124E06`, alive B1 low nibble 0..14 | CONFIRMED |
| Period | 2 ticks (~10 ms) | CONFIRMED |
| Fault index | 0xF5; cal `0x8032E6DC`; MB `0x80231BEC` | CONFIRMED |
| Sender | **ICM / LDM** (public DTC cluster 33.1.4 is ICM, not DSC) | ANALYSIS |
| Public name | Anforderung Radmoment Antriebstrang Summe FAS | CONFIRMED (public DTC text) |
| PT-CAN mirror | **None known** in owned captures / RX tables | ANALYSIS — check ZGM if needed |

### 1.2 Signals (Intel / LSB-first)

| Start bit | Len | Type | RAM | Scale / SNA | Role | Label |
|---|---|---|---|---|---|---|
| 0 | 8 | crc8_e2e | d0006F8A | — | E2E CRC | CONFIRMED |
| 8 | 4 | alive 0..14 | d00059F6 | — | Alive | CONFIRMED |
| 12 | 4 | u4 qualifier | **d00070E4** | SNA **15** | FAS request qualifier | CONFIRMED |
| 16 | 16 | u16 torque | **d00030F4** | **1 Nm/bit, offset −32000**; SNA **0xFFFF** | Requested wheel-torque sum | CONFIRMED |
| 32 | 12 | u12 companion | **d0005ACE** | SNA **4095**; idle **0xE00** (3584) | Companion / limit field | CONFIRMED |

### 1.3 Shadow / consumer chain [CONFIRMED by disassembly]

1. Shadow stub `0x80184362` (task `0x800dd162`):
   - `d00030F4 → d00030F0`, `d0005ACE → d0005ACC`, `d00070E4 → d00070E3`
2. Copy layer `FUN_80160CF0` region:
   - `0x801612b4`: `d00030F0 → d0003688` (FAS request into arbiter)
   - `0x801611f0`: `d0005ACC → d0004058`
3. Level-2 monitor `0x800f4698`: `d0005ACC → abs 0xd00002BC` (Fid `a1−0x7132`, subst 0xFFF)
4. **FAS arbiter** `FUN_800ac458` / `FUN_800ac474` (init `FUN_800ac3a4`, runs to ~`0x800ad0c0`):
   - State `d0006E5E` ∈ {0..3}; flag `d0006E61 = (state ∈ {2,3})`
   - Flags `d0006E65`: bit0/1 = FAS torque request active (two paths on `d0007711`); bit2 = limit request; bit3 derived
   - Request `d0003688` optionally offset by `d00036E4` (`d0007678` bit7); clamped by gain `0x2000 ± d0005FE0`
   - Outputs: `d0003010` = FAS target; `d0003014` = FAS max limit (else 500000 = “no limit”); `d0003650` = processed request
   - Status enum **`d00075DB`** (values seen in code: 0,1,2,3,7,8,9,10,11) — acceptance / realisation status [ANALYSIS]
   - Override / abort **`d00075DC`** — driver-pedal override / FAS not available [ANALYSIS]
5. Packer path: arbiter → `d00075DB`; copy `0x80161ed0` / reverse `0x80164652` keeps `d0006E5F` ↔ `d00075DB` in sync for FR vs PT-CAN TX.

No stalk bits, set-speed, resume, cancel, +/- or distance signals appear anywhere in this chain [CONFIRMED].

---

## 2. What is *not* cruise RX on the DDE

| Candidate | Finding | Label |
|---|---|---|
| PDU35 0x100 (four 2-bit enums → d0007019/15/14/18) | Shadows in `FUN_80169b70`; **not** linked into the FAS arbiter | ANALYSIS — leave open |
| PDU33 0xEA / PDU36 0x101 / PDU22 0x87 | No arbiter edge | ANALYSIS |
| PT-CAN unknown RX (0x2F9, 0x3B0, 0x8A, …) | No cruise-specific decode found | ANALYSIS |
| Strings GRA / Tempomat / Cruise / ACC / FGR / DCC | **Absent** from DDE flash | CONFIRMED |

---

## 3. DDE TX that reports FAS / cruise-related status

### 3.1 PT-CAN 0xD9 (and PT-CAN2 0xD9) — pedal + FAS status

| Bus | ID | Rate | Bit56 (B7 low nibble) RAM | Label |
|---|---|---|---|---|
| PT-CAN (cap bus 3) | **0xD9** | ~20 ms | **`d00075DB`** (vid 331) | CONFIRMED |
| PT-CAN2 (cap bus 2) | **0xD9** | ~10 ms | **`d0006E5F`** (synced copy of status) | CONFIRMED |

Same PDU also carries accelerator pedal (bits 16/32, 0.025 %/bit) — required for ICM pedal-override detection [ANALYSIS].

### 3.2 FlexRay 0x28 PDU52 bit56

| Field | Value | Label |
|---|---|---|
| FID / PDU | 0x28 / PDU52 | CONFIRMED (flash TX table) |
| Bit56 u4 | **`d0006E5F`** | CONFIRMED |
| Meaning | Same FAS status enum as PT-CAN 0xD9 bit56 | ANALYSIS |

### 3.3 Capture observation (no cruise engaged) — NEEDS CAPTURE for engagement

| Log | FR 0x21 (qual, tq, companion) | 0xD9 B7&0xF |
|---|---|---|
| F11_4log_ignonoff | (0, 0, 3584) dominant; rare SNA | always **0** |
| F11_4log_3krev | same idle; SNA rare | **0**, briefly **1** while pedal > 0 |
| F11_4log_r10md10m | same idle | always **0** |

Status **1** with pedal pressed is consistent with “driver demand / not FAS” rather than cruise-active [HYPOTHESIS]. Full enum ↔ ICM mapping needs a capture with stalk Set/Resume and ACC distance changes.

---

## 4. High-level state machine pointers (FAS arbiter)

| Symbol | Addr / RAM | Role | Label |
|---|---|---|---|
| Init | `FUN_800ac3a4` | `d0006E5E=1`, `d0003014=500000`, `d0003010=0` | CONFIRMED |
| Body | `FUN_800ac458` / `FUN_800ac474` → ~`0x800ad0c0` | Arbitrate FAS vs base demand; ramp via `FUN_80087ad4` | CONFIRMED |
| State | `d0006E5E` | 0..3; Fid-gated (`a1−0x6b36/30/38/32`) + mode `d0007711` | ANALYSIS |
| Active flags | `d0006E65` bits 0..3 | Request / limit active | ANALYSIS |
| Status out | `d00075DB` | Packed to 0xD9 / FR 0x28 | CONFIRMED packing |
| Override | `d00075DC` | Pedal hysteresis vs `d0003694` | ANALYSIS |
| DSM | `FUN_800e2b60` ids `a1−0x7580`, `a1−0x757e` | Fault report | CONFIRMED call sites |

---

## 5. EGS (8HP / SH725xx) — cruise / GRA / FAS / ACC related

Sources: `/workspace/8hp-firmware/ecu.a2l` (hybrid GA8P75HZ build), `egs_8hp_map.md`, `egs_8hp_ids.csv`,
`/workspace/8hp45-dump/egs_8hp45_cantcu_can_map.md`.

### 5.1 Terminology caveat [CONFIRMED]

In the ZF/Bosch A2L, **“Cruise”** often means the TCU OS run mode (`Interfi.st_OsMode`: 0 Init / 1 Stdby / **2 Cruise** / 3 Limphome), **not** vehicle cruise control. Treat `*Cruise*` OS/runtime names carefully.

### 5.2 Real cruise / ACC / FAS artefacts in the A2L [CONFIRMED]

| A2L object | Meaning |
|---|---|
| `COMPU_VTAB _CNV_S_8_Enum_EFGR_74_S2` | **FGR** (Fahrgeschwindigkeitsregler) states: `FGR_PASSIV`, `FGR_AKTIV_KONSTANTFAHRT`, `FGR_ACC_REGELUNG_STANDARD`, `FGR_AKTIV_WIEDERAUFNAHME`, `FGR_ACC_REGELUNG_ERH_DYNAMIK`, `FGR_AKTIV_SETZEN_BESCHL`, `FGR_ACC_REGELUNG_ABSCHALTUNG`, `FGR_AKTIV_VERZOEGERN` |
| `S_FGR_FFG` @ `0xFFF9122F` | FGR status measurement (ubyte) |
| `ZFFSAB._.LDMDesiredTorqueIn.iv_m_LDM` | **Desired LDM torque from CAN**, scale **1 Nm, offset −32000** (same as DDE FR 0x21) |
| `…iv_FASMode` / `sv_LDMMode` / `sv_LDMMode2` | LDM / **“ACC/DCC Schaltprogram Zustand”** |
| `iIaeSigna.fas_aktiv_iae` | “Fahrerassistenzsystem aktiv” |
| `KL_M_KRIECHMAX_FAS_IAE` | Max creep torque when FAS active |
| `GETCAN_RX_RQ_OPMO_GRDT_DRS` | Engine ECU → gearbox: **FAS-requested operating modes** |
| `GETCAN_RX_STATUS_TAR_OPMO_PT_GRB` | Target OPMO status (PT gearbox) |
| `GETCANFA_RX_ST_BRG_DV` | Brake intervention by driver **or** FAS |
| ParkManeuverAssistant `iv_FASMode` / `iv_FASPos` / `sv_FASGearbox` | PMA (park assist) FAS handshake — **not** Tempomat stalk |

### 5.3 What EGS publishes / consumes on the wire (owned F11 map)

From `egs_8hp_ids.csv` / `egs_8hp_map.md` — **no dedicated GRA/Tempomat TX ID** was identified on PT-CAN / PT-CAN2 [ANALYSIS]. EGS TX of interest remains gear/turbine (`0x1AF`, `0x3FD`, `0x39A`, …).

EGS **consumes** (available on its buses; “consumed” mostly HINT via DTC text):

| ID | Role vs cruise | Label |
|---|---|---|
| **0xD9** (DDE) | Pedal (+ FAS status nibble on B7) | CONFIRMED present; ANALYSIS whether EGS reads B7 |
| **0x0A5** (DDE) | Crank rpm / torque | HINT consumed |
| **0x1A1 / 0x254 / 0x0EF / 0x173** | Speed / brake (DSC→CAN) | HINT |
| **0x199/19A/19F/0x302** | ICM accel/yaw/steer | HINT (ICM DTCs) |
| LDM desired torque + ACC/DCC mode | Expected by A2L `LDMDesiredTorqueIn` | CONFIRMED in A2L; **CAN ID not mapped** on owned F11 logs → **NEEDS CAPTURE / further map** |

Hybrid-only **Radmoment** recuperation signals (`SollRadmomentAntriebstrangSummeRek_Getr_Hybrid`, etc.) are present in this hybrid A2L; they are EME/SME-side, not F11 520d diesel cruise [ANALYSIS].

### 5.4 EGS implication for ZV / TCU emulation

With engine **and** gearbox removed (CAS/BDC stay):

- Body does **not** appear to need a synthetic “EGS cruise TX” frame beyond the existing EGS gear/NM set [ANALYSIS].
- If a real 8HP (or open TCU) remains, it expects **LDM torque + ACC/DCC shift-program state** and **FAS OPMO** from the engine path — the ZV should either (a) republish those once the CAN ID is known, or (b) leave them inactive/SNA so shift strategy stays driver-pedal-based [HYPOTHESIS].
- Do **not** invent CAS/BDC frames; ZV should not synthesise `0x12F`.

---

## 6. EV / ZV recommendation

| Action | Detail | Label |
|---|---|---|
| **Honour RX** | **FR 0x21 PDU2** FAS wheel-torque (1 Nm −32000, qual bits 12–15, companion bits 32–43). When qual shows active (≠0/≠15), use as cruise/ACC **torque target or ceiling**. Needs Pico/FlexRay bridge. | CONFIRMED frame; ANALYSIS policy |
| **No stalk emulate** | Set / Resume / Cancel / +/- / distance / set-speed stay with ICM. ZV need not fake them. | ANALYSIS |
| **Keep TX** | Pedal **0xD9** (both buses) with plausible **B7 status nibble** (`d00075DB` semantics); FR **0x28** PDU52 bit56; wheel torques **0x29/0x3D/0x28**; **DT_PT_2 / 0x3F9**; rest of MUST set in `dde_ptcan_emulation_plan.md`. Lets ICM close LDM and see pedal override. | CONFIRMED need for 0xD9/pedal; ANALYSIS for status values under engagement |
| **EGS** | No extra cruise TX required for body. If emulating TCU shift logic, plan for LDM/FAS OPMO inputs once IDs are captured. | ANALYSIS |
| **Do not invent** | CAS/BDC frames (esp. `0x12F`); stalk/set-speed frames. | CONFIRMED constraint |

---

## 7. Open / NEEDS CAPTURE

1. Drive log with **GRA Set / Resume / Cancel / +/-** and (if equipped) **ACC distance** — decode FR 0x21 qualifier≠0 and 0xD9 / FR 0x28 status enum values under engagement.
2. Confirm whether ZGM mirrors 0x21 onto any PT-CAN ID (would help a CAN-only ZV).
3. Map EGS `LDMDesiredTorqueIn` / `GETCAN_RX_RQ_OPMO_GRDT_DRS` to concrete CAN IDs on F11.
4. Pin PDU35 `0x100` enums if any ICM cruise-adjacent meaning appears under engagement.
5. Full enum table for `d00075DB` (0,1,2,3,7–11) ↔ ICM acceptance codes.

---

## Paragraph for Sam

Cruise on this F11 DDE is not a stalk decoder — it is a **torque follower**. The ICM/LDM owns Set/Resume/Cancel/set-speed/display; the DDE only receives **FlexRay 0x21 PDU2** (*Anforderung Radmoment Antriebstrang Summe FAS*: 1 Nm/bit −32000, 4-bit qualifier, 12-bit companion, E2E data ID 0x03) and arbitrates it in `FUN_800ac458` into target/limit RAMs, then reports a status nibble on **PT-CAN 0xD9 B7** (`d00075DB`) and **FR 0x28 PDU52 bit56** (`d0006E5F`). Owned logs never engaged cruise (qualifier stayed idle; status 0, or 1 with pedal) — mark engagement **NEEDS CAPTURE**. For the ZV: honour 0x21 as the ACC/cruise torque ceiling when the qualifier is active, keep publishing pedal + status on 0xD9/0x28 so the ICM can run LDM and see override, and do not invent stalk or CAS/BDC frames. The 8HP A2L confirms the gearbox also knows FGR/ACC states and expects an LDM desired-torque + ACC/DCC shift-program input (same −32000 Nm scaling) plus FAS OPMO from the engine path, but no dedicated EGS “cruise TX” showed up on the owned CAN map — so TCU emulation should not invent one; only feed LDM/FAS inputs if/when those IDs are captured.
