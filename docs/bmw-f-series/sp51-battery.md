# BMW SP51 (Gen5 PHEV) battery on the ZombieVerter

`BMS_Mode = 8 (BMW_SP51)` drives a BMW Gen5 PHEV high-voltage battery
(SP51, Preh EA443 SME) over classic CAN at 500 kbit/s. The pack has its
own contactors and precharge. The ZombieVerter plays the part of the
car's EME: it asks the pack to close and open, and if needed it sends
the few car frames the pack wants to see before it will talk.

Everything here comes from Ian's bench work (`sp51ctl.py`, the close_v4
and close_v7 playbacks, `SP51_CONTACTOR_CLOSE_GUIDE.md`). The pack closed
on the bench with exactly these frames. It has **not** been run on a
ZombieVerter yet, so the first tests should be on the bench with no load.

**Danger:** once closed, the HV terminals carry the full pack voltage
(about 380 V DC). Treat the output as live whenever 12 V is on and the
ZV is running.

## Wiring

| Pack LV pin | Signal | Connect to |
|---|---|---|
| 1 | KL30 | +12 V, fused 5 A |
| 3 | 30C | +12 V. Without it the pack stays silent. |
| 4 | Wake | +12 V |
| 7 / 8 | HVIL loop | Bridge 7 to 8, or route through your HVIL chain. Open HVIL = no CAN at all. |
| CAN-H / CAN-L | | The ZV CAN port chosen with `BMSCan` |

(Pin roles are from the owner's diagram of connector A332*11B.)

- **Switch the pack's 12 V with the ZV's 12 V.** The pack wants CAN
  traffic within about 5 s of its 12 V coming on, and once it has
  started talking it latches its CAN off after about 1.2 to 1.8 s of
  silence. Only a 12 V power cycle (off for 5 s or more) brings it back.
  The ZV sends from power up and never pauses, so powering both together
  is the simple way to get this right.
- **CAN bus:** 120 Ohm at each end. It is not known whether the pack has
  internal termination, so measure H to L with the power off (about
  60 Ohm is right).
- **Contactor outputs:** the pack does its own precharge and main
  contactors. Do not wire ZV precharge or main contactor relays into the
  pack's HV path. The ZV outputs still switch as normal (the module
  follows them, see below), so they can stay unconnected or drive an
  indicator.
- 12 V supply: the contactor coils draw more than the idle current, allow
  3 A or more.

## Parameters to set

| Parameter | Set to | Why |
|---|---|---|
| `BMS_Mode` | 8 = BMW_SP51 | Selects this module |
| `BMSCan` | the CAN port the pack is on | All pack frames go out on this port only |
| `SP51Supp` | On (default) when the pack has its own bus; Off when it shares a bus with a car that already sends 0x12F, 0x1A1, 0x328 and friends | See below |
| `ShuntType` | 0 = None (or 1 = ISA if you fit one) | With 0 the module fills in `udc`, `udc2`, `idc`, `power` and `SOC` from the pack |
| `udcsw` | leave it | The module writes it: pack voltage minus 30 V, or 500 V when there is no pack data |

`SP51Supp` (id 166) switches the support frames: 0x12F (terminal status,
KL15 on), 0x1A1 (speed, standing still), 0x328 (car clock), 0x330, and
0x3A0, 0x3CA, 0x2CA, 0x3E8, 0x510, 0x512 once a second. These are the car
frames from the bench playback. They are never sent when the pack is on
the same CAN port as the BMW F11 vehicle class (`BMSCan = VehicleCan` and
`Vehicle = BMW_F11`), because on the car's PT-CAN the real car already
sends them. 0x10B and 0x53A (the EME's close request) are always sent.

## How a close works

1. **Wake** (`SP51State` 0): from power up the ZV sends 0x10B OPEN,
   0x53A state A and the support frames. The pack answers within about
   0.12 s with 0x112 every 20 ms. If nothing at all comes back within
   3 s, the ZV goes quiet for 1 s and starts again once, with fresh
   counters. It never goes quiet after the pack has answered.
2. **Standby** (1) until 0x112 has run for 5 s without a gap over 0.1 s
   and the last three 0x1FA frames said `0x45` (open, ready). Then
   **Ready** (2).
3. The ZV asks for HV when it is in precharge or holds its precharge or
   main contactor output on (`prec_out` or `dcsw_out`). From Ready that
   starts **Request** (3): 0x53A state B for 2 s.
4. Last check: 0x112 present, 0x1FA `0x45`, the pack is not asking to
   open, pack voltage over 200 V. Then **Closing** (4): 0x10B CLOSE with
   0x53A state C for two frames, then D.
5. The pack goes to state 5 (precharge) within about 80 ms and state 6
   (closed) about 1.8 to 1.9 s later on the first close after power up,
   about 0.6 s on later ones. That is **Closed** (5).
6. When the ZV drops its request (key off, after its usual 2.5 s delay;
   precharge fail; over voltage) the module sends OPEN and state A, and
   goes back to Standby.

**Fault** (6) opens the pack and stays there until the ZV drops its
request. `SP51Fault` says why: 1 PackLost (0x112 missing over 0.5 s while
closing or closed), 2 OpenReq (the pack asked to open), 3 PrecheckFail,
4 CloseTimeout (no state 6 within 5 s), 5 LeftClosed (out of state 6 for
over 1 s). **Lost** (7) means the pack was talking and stopped: check
HVIL and 12 V, then power cycle the pack's 12 V.

### ZV precharge

The ZV's own precharge logic is used as is. While the pack is not closed
the module reports `udc = 0`, so the ZV stays in precharge. Once the pack
reports state 6 it reports `udc` = pack voltage, which is above `udcsw`,
and the ZV moves on to Run or Charge.

The ZV gives precharge 5 s. With the 2 s of state B and the pack's own
1.9 s first precharge, a close takes about 4 s from key on, which leaves
about 1 s to spare. **If the key is turned within about 4 to 5 s of the
ZV powering up** the pack has not finished its 5 s settle, the ZV times
out (PchFail) and the pack is never asked to close. Turn the key off and
on again. The host check covers this case.

## Values

| Value | Id | Meaning |
|---|---|---|
| `SP51State` | 2142 | Step above |
| `SP51Fault` | 2143 | Last abort reason |
| `SP51HvSt` | 2144 | Pack contactor state from 0x1FA: 4 open, 5 precharge, 6 closed |
| `SP51ULink` | 2145 | Link voltage after the contactors, 0x112 b7 (about 4 V per bit, a guess) |
| `SP51UMax` | 2146 | Pack maximum charge voltage, 0x2F5 |
| `SP51UMin` | 2147 | Pack minimum discharge voltage, 0x2F5 |
| `SP51IDchg` | 2148 | Pack discharge current limit, 0x2F5 |

The usual battery values are filled too: `udc2`, `idc`, `power` (from
0x112), `BMS_Tmin` and `BMS_Tmax` (cell temperatures from 0x1FA),
`BMS_ChargeLim`, `BMS_MaxInput` and `BMS_MaxOutput` (kW, from 0x40D),
and `SOC` from 0x432 only when it is a valid 0 to 100. Cell voltages are
only available over UDS, so `BMS_Vmin` and `BMS_Vmax` stay 0.

## What is not certain yet

- Current limits: the scaling of 0x2F5 charge current is a guess, so
  `BMS_ChargeLim` is the lower of that and the 0x40D charge power divided
  by pack voltage, and 0 when there is no pack data (that ends AC
  charging).
- 0x40D: which words are short and long term limits is a guess, so the
  lower of the two is used.
- 0x1FA cell temperatures (b6, b7 minus 50) and 0x112 link voltage are
  analysis, not confirmed.
- 0x432 SOC reads 0xFF (invalid) on the bench.
- The sign of the 0x112 current (charge or discharge positive) is not
  confirmed.
- What the pack does if 0x10B stops while closed is not tested. Always
  power the pack's 12 V from the same switched supply as the ZV.

## Host check

`test/bmw_sp51/run.sh` builds the module on the PC with a simple pack
model and compares every frame it sends, byte for byte (CRC and alive
counters included), with Ian's `sp51_close_v7_continuous.csv`, plus abort
scenarios (0x112 lost, pack open request, no answer, early key on,
`SP51Supp` off, battery on the F11 PT-CAN):

    SP51_CSV=/path/to/sp51_close_v7_continuous.csv test/bmw_sp51/run.sh
