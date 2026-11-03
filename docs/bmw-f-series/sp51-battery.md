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
| `udcsw` | leave it | The module writes it: pack voltage minus 30 V once the pack is closed, 500 V otherwise |

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
3. The pack follows the ZV opmode:

   | ZV opmode | Pack |
   |---|---|
   | Off, PchFail | open (0x10B OPEN, 0x53A A). Sending carries on. |
   | Precharge | run the close sequence below |
   | Run, Charge (and Preheat, 12VMaintain) | stay closed |

   Precharge from Ready starts **Request** (3): 0x53A state B for 2 s.
4. Last check: 0x112 present, 0x1FA `0x45`, the pack is not asking to
   open, pack voltage over 200 V. Then **Closing** (4): 0x10B CLOSE with
   0x53A state C for two frames, then D.
5. The pack goes to state 5 (precharge) within about 80 ms and state 6
   (closed) about 1.8 to 1.9 s later on the first close after power up,
   about 0.6 s on later ones. That is **Closed** (5).
6. When the ZV goes to Off or PchFail the module sends OPEN and state A
   at once, and goes back to Standby. The ZV's 2.5 s delayed release of
   its own contactor output does not apply to the pack, and an over
   voltage trip that only drops the ZV outputs does not open the pack.

**Fault** (6) opens the pack and stays there until the ZV goes back to
Off (or PchFail). `SP51Fault` says why: 1 PackLost (0x112 missing over 0.5 s while
closing or closed), 2 OpenReq (the pack asked to open), 3 PrecheckFail,
4 CloseTimeout (no state 6 within 5 s), 5 LeftClosed (out of state 6 for
over 1 s). **Lost** (7) means the pack was talking and stopped: check
HVIL and 12 V, then power cycle the pack's 12 V.

### ZV precharge

The ZV's own precharge logic is used as is: it waits for `udc` to reach
`udcsw`. 0x112 carries two voltages:

- **U_Pack** (bytes 2-3, 0.1 V, confirmed): the cell stack side, before
  the contactors. It reads the full 380 V even with the pack open. This
  goes to `udc2` (battery voltage, as in the Leaf class).
- **U_Link** (byte 7, about 4 V per bit, analysis): the output side after
  the contactors, 0 V when open, about 380 V when closed. With
  `ShuntType 0` this goes to `udc` (the bus side voltage the precharge
  check uses). With an ISA shunt the shunt sets `udc`.

`udcsw` is held at 500 V until the pack reports state 6 (closed), then
set to pack voltage minus 30 V. So the ZV only moves on to Run or Charge
once the pack says closed **and** the link voltage is up. If the 4 V per
bit guess is wrong low, the ZV will report PchFail rather than run.

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

The usual battery values are filled the way the Leaf class does:

- With `ShuntType 0`: `udc` (link side), `udc2` (pack side), `idc`,
  `power`, and `SOC` from 0x432 only when it is a valid 0 to 100.
- Always: `udcsw`, `BMS_Tmin`, `BMS_Tmax` and `BMS_Tavg` (cell
  temperatures from 0x1FA; the average is the middle of min and max),
  `BMS_ChargeLim` (A), and `BMS_MaxInput` and `BMS_MaxOutput` (kW, from
  0x40D).

Left unset (not on CAN, or not decoded): `BMS_Vmin` and `BMS_Vmax` (cell
voltages are only on UDS), `BMS_IsoMeas`, SOH, and a ZV discharge current
limit (there is no generic one; the pack's is shown as `SP51IDchg`).

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
