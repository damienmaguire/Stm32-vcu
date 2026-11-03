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
| 4 | Wake | +12 V (from our guide, untested) |
| 7 / 8 | HVIL loop | Bridge 7 to 8, or route through your HVIL chain. Open HVIL = no application CAN (0x112 etc.); UDS still answers. |
| CAN-H / CAN-L | | The ZV CAN port chosen with `BMSCan` |

(Pin roles are from the owner's diagram of connector A332*11B.)

- **Switch the pack's 12 V with the ZV's 12 V.** The pack wants CAN
  traffic within about 5 s of its 12 V coming on, and once it has
  started talking it latches its CAN off after about 1.2 to 1.8 s of
  silence (a ZV reset or reflash, or a bus-off on that port, all count).
  Only a 12 V power cycle (off for 5 s or more) brings it back.
  The ZV sends from power up and never pauses, so powering both together
  is the simple way to get this right.
- **Latched pack:** a latched pack sends no 0x112 but still answers UDS,
  exactly like an open HVIL. If the ZV restarts while the pack stays
  powered, `SP51State` sits in Wake (0) or Lost (7). Wake for more than
  5 s means: check HVIL, then cycle the pack's 12 V. An optional relay
  (or a ZV output, via IOMatrix, driving one) in the pack's KL30/30C feed
  makes that cycle easy; the module does not drive one yet.
- **CAN bus:** 120 Ohm at each end. It is not known whether the pack has
  internal termination, so measure H to L with the power off (about
  60 Ohm is right).
- **Shared CAN port:** the ZV has two CAN ports, so the pack usually
  shares one with the inverter or charger. Check that no other node sends
  the pack's receive ids (0x10B, 0x12F, 0x1A1, 0x328, 0x330, 0x3A0, 0x3CA,
  0x2CA, 0x3E8, 0x510, 0x512, 0x53A, and also 0x380 and 0x03C), and that
  the pack's own ids (0x112, 0x1FA, 0x239, 0x2F5, 0x40D, 0x423, 0x431,
  0x432, 0x4AD, 0x4B0, 0x587 and more) don't clash with the inverter's. A
  bus-off is a silence and latches the pack.
- **Contactor outputs:** the pack does its own precharge and main
  contactors. Do not wire ZV precharge or main contactor relays into the
  pack's HV path. The ZV outputs still switch as normal (the module
  follows them, see below), so they can stay unconnected or drive an
  indicator.
- 12 V supply: the contactor coils draw more than the idle current, allow
  3 A or more (from our guide, untested).

## Parameters to set

| Parameter | Set to | Why |
|---|---|---|
| `BMS_Mode` | 8 = BMW_SP51 | Selects this module |
| `BMSCan` | the CAN port the pack is on | All pack frames go out on this port only |
| `SP51Supp` | On (default) when the pack has its own bus; Off when it shares a bus with a car that already sends 0x12F, 0x1A1, 0x328 and friends | See below |
| `ShuntType` | 0 = None (or 1 = ISA if you fit one) | With 0 the module fills in `udc`, `udc2`, `idc`, `power` and `SOC` from the pack |
| `SP51Settle` | 20 s (default) | How long 0x112 must run without a gap before a close may start. 20 s is what every proven bench close had; 5 s is untested (5 to 60 s) |
| `udcsw` | leave it | Managed by this module, overwritten 10 times a second: pack voltage minus 30 V once the pack is closed, 500 V otherwise |

`SP51Supp` (id 166) switches the support frames: 0x12F (terminal status,
KL15 on), 0x1A1 (speed, standing still), 0x328 (car clock), 0x330, and
0x3A0, 0x3CA, 0x2CA, 0x3E8, 0x510, 0x512 once a second. These are the car
frames from the bench playback. 0x12F is a static KL15 on (the pack does
not decode it; it may only be the wake) and 0x1A1 a static standstill
(b4 0x81). They are never sent when the pack is on
the same CAN port as the BMW F11 vehicle class (`BMSCan = VehicleCan` and
`Vehicle = BMW_F11`), because on the car's PT-CAN the real car already
sends them. 0x10B and 0x53A (the EME's close request) are always sent.
**Warning:** that shared PT-CAN setup has never been tried on a real
pack, and it puts the pack's own frames on the car's PT-CAN. Prefer a
separate port.

## How a close works

1. **Wake** (`SP51State` 0): from power up the ZV sends 0x10B OPEN,
   0x53A state A and the support frames. The pack answers within about
   0.12 s with 0x112 every 20 ms. If nothing at all comes back within
   3 s of ZV power up, the ZV goes quiet for up to 1 s and starts again
   once, with fresh counters. If the pack answers during that second the
   ZV talks again at once. It never goes quiet after the pack has
   answered.
2. **Standby** (1) until 0x112 has run for `SP51Settle` (20 s) without a gap over 0.1 s
   and the last three 0x1FA frames said `0x45` (open, ready). Then
   **Ready** (2).
3. The pack follows the ZV's own contactor outputs (`prec_out` and
   `dcsw_out`), so it opens exactly when external contactors would,
   delays included:

   | ZV | Pack |
   |---|---|
   | Precharge | run the close sequence below |
   | Run, Charge (and Preheat, 12VMaintain) | stay closed |
   | Key off / charge end (Off) | stay closed for the ZV's 2.5 s shutdown delay, then open |
   | PchFail | open |
   | Outputs dropped with no delay (e.g. a stationary over voltage trip during the shutdown delay) | open on the next 10 ms tick |

   Open means 0x10B OPEN and 0x53A state A; sending carries on. The
   Precharge opmode is checked as well as `prec_out`, because the ZV
   turns `prec_out` on 250 ms into Precharge and the sequence should
   start at once.

   Precharge from Ready starts **Request** (3): 0x53A state B for 2 s.
4. Last check: 0x112 present, 0x1FA `0x45`, the pack is not asking to
   open, pack voltage over 200 V. Then **Closing** (4): 0x10B CLOSE with
   0x53A state C for two frames (400 ms), then D.
5. The pack goes to state 5 (precharge) within about 80 ms and state 6
   (closed) about 1.8 to 1.9 s later on the first close after power up,
   about 0.6 s on later ones. That is **Closed** (5).
6. On key off the ZV goes to Off and keeps its contactor outputs on for
   2.5 s while the inverter, DC-DC, charger and heater shut down, so no
   current is flowing when the contactors open. Then the module sends
   OPEN and state A and goes back to Standby.

Only faults on the pack side open it at once, under load if need be:
0x112 missing for 0.5 s, or the pack asking to open. The pack's open
requests are three 2-bit fields in 0x112, 1 = no request: b5 bits 6-7
(normal open), b6 bits 0-1 (instant open) and bits 2-3 (fast open). b6
reads 0x75 in every log. Any other value, 0 included, counts as a
request to open.

**Fault** (6) opens the pack and stays there until the ZV contactor
outputs are off. A fault, or the pack going Lost, posts the ZV error
`BMSCOMM`. That only sets `lasterr` and the error log: nothing in the ZV
acts on it, so the opmode stays Run (or Charge) with the pack open until
the key goes off or charging ends. The charge and discharge limits drop
to 0 on the same 10 ms tick as the abort, and are 0 whenever the module
is not in Closed or 0x1FA does not report state 6. `SP51Fault` says
why: 1 PackLost (0x112 missing over 0.5 s while closing or closed),
2 OpenReq (the pack asked to open), 3 PrecheckFail, 4 CloseTimeout (no
state 6 within 5 s), 5 LeftClosed (out of state 6 for over 1 s),
6 NotReady (no CLOSE within `SP51Settle` + 15 s of the ZV asking, see
the precharge timeout below). **Lost** (7) means the pack was talking and stopped: check
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

**Precharge timeout.** The ZV gives precharge 5 s (`PRECHARGE_TIMEOUT`
in `stm32_vcu.cpp`, counted on the 1 s RTC, so 5 to 6 s). The module
holds that timeout through `ReadyForPrecharge()` (a BMS hook, true for
every other battery): it is false while the module settles, does its
checks and sends 2 s of 0x53A state B, and true from the CLOSE on. So the
ZV's 5 s cover only the close and the pack precharge (1.8 to 1.9 s on
the first close after the pack's 12 V came on, about 0.6 s later,
possibly longer into a real inverter). The ZV stays in Precharge while
held; a cold start (pack and ZV powered together, key turned at once)
reaches Run about `SP51Settle` + 4 s after key on, with no PchFail.

The hook is true again, so the ZV times out to PchFail as before, when:

- the module aborts (Fault, e.g. PrecheckFail) or the pack goes Lost
- the ZV has been in Precharge for `SP51Settle` + 15 s (35 s by default)
  without a CLOSE: the module aborts with `SP51Fault` 6 NotReady (and
  posts `BMSCOMM`), and PchFail follows 5 to 6 s later
- the battery has no CAN port assigned

Letting go of the key during the hold goes to Off as before; nothing was
asked of the pack, so it just stays open.

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
  `power`, and `SOC` from 0x432 only when it is a valid 0 to 100. It has
  read 0xFF in every log, so SOC stays unset in practice.
- Always: `udcsw`, `BMS_Tmin`, `BMS_Tmax` and `BMS_Tavg` (cell
  temperatures from 0x1FA; the average is the middle of min and max).
- Only while the module is in Closed and the pack reports closed (0x1FA
  state 6), 0 otherwise:
  `BMS_ChargeLim` (A), `BMS_MaxInput` and `BMS_MaxOutput` (kW, from
  0x40D), `SP51IDchg`.

Left unset (not on CAN, or not decoded): SOC (no valid SOC on CAN; per
cell SOC is only on UDS, 0x239 b1-2 may be an energy figure in Wh but is
unconfirmed), `BMS_Vmin` and `BMS_Vmax` (cell
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
  confirmed. Raw 0xFFF0 and up is "not valid" and ignored (0xFFFD shows
  up once at most closes and would read as 5734 A); so are U_Pack 0xFFFF
  and link 0xFF.
- 0x40D word 3 has a constant 0x80 high byte, possibly a flag. Harmless
  with the lower-of-two rule; recheck under load.
- Settle time: 5 s has not been tried on the bench, hence the 20 s
  default.
- ZV over voltage trip (`udc` over `udclim`, in `utils::ProcessUdc`):
  in Run, Charge and Precharge the mode control later in the same 10 ms
  task turns the outputs straight back on and restores the opmode. So
  the trip does not open external contactors or this pack; only during
  the Off shutdown delay does it open them early. This is existing ZV
  behaviour and is not changed here.
- What the pack does if 0x10B stops while closed is not tested (it does
  log a fault, CAD9A6). Treat it as "may stay closed if the ZV stops" and
  always power the pack's 12 V from the same switched supply as the ZV.

## Host check

`test/bmw_sp51/run.sh` builds the module on the PC with a simple pack
model and compares every frame it sends, byte for byte (CRC and alive
counters included), with Ian's `sp51_close_v7_continuous.csv`, plus abort
scenarios (key off with the 2.5 s hold, outputs dropped with no delay,
0x112 lost, pack open request on b5 and on b6, a 0xFFFD current, no
answer, the pack answering during the wake retry silence, a cold key on
held until the CLOSE, the same with a plain BMS (not held, PchFail), a
pack that never gets ready, a lost pack, a failed last check and a key
off during the hold, `SP51Settle` 5, `SP51Supp` off, battery on the F11 PT-CAN):

    SP51_CSV=/path/to/sp51_close_v7_continuous.csv test/bmw_sp51/run.sh
