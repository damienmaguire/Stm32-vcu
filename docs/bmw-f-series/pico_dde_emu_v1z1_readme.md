# Pico DDE emulator v1z.1: README (copy)

This is the README that ships with the Pico firmware **dde-emu v1z.1** (pico-flexray, commit 3f3df48), copied here unchanged because the ZV's BMW_F11 code and test harness are matched to that release.

Notes for the ZV side (since PR #277):

- The ZV no longer has a second-sender check. The lines below that say "F11Fault = 1 = a second sender" are out of date: `F11Fault` 1 now means "Pico lost". The real DDE/EGS must still be off PT-CAN.
- With `F11Pico` = 1 the ZV sends the private frames 0x304-0x307, so the Pico uses source 1 (PRIVATE) below. With `F11Pico` = 0 it falls back to source 2 (PT-CAN), as the README describes.
- The Pico must run with its private TX on (`--can-tx`, or the standalone image) for the ZV to see it. Otherwise the ZV reports "Pico lost" when `F11Pico` = 1.

```text
DDE emulator v1z.1 - ZombieVerter link  (pico-flexray, local branch dde-emu-zv, on top of v1h.3 = 63eb320)
=========================================================================================================

In one paragraph
----------------
v1z.1 is the v1h.3 FlexRay DDE/EGS emulator for the V2 board (RP2350 + MCP251863 on
BMW PT-CAN), changed so that it works NEXT TO a ZombieVerter (ZV) running the BMW_F11
vehicle module (Stm32-vcu PR #276). The ZV now sends every BMW PT-CAN frame the engine
and gearbox used to send (0x3F9, 0x0A5, 0x3FD, 0x39A, ...). This Pico sends NONE of them.
It keeps doing all the FlexRay work exactly as v1h.3 (same timing fixes, 0xE6 only in even
cycles, same E2E) and copies the ZV's engine and gear state onto FlexRay, so both buses
tell the car the same story. On PT-CAN it only sends its own private frames 0x308-0x30C.

NOTHING in this release has run on hardware yet. It was checked in host simulation only
(see "What was tested"). Do the car test below in order and stop at the first surprise.
In this build the Pico never sends ANY BMW PT-CAN ID: every frame is checked against
the list 0x308-0x30C right before it goes to the CAN chip.

Firmware version: dde-emu-zv-v1z.1 (standalone: dde-emu-zv-standalone-v1z.1)
Logger version:   v1z.1 (also accepts the v1h.3 firmware without a mismatch warning)
Copy fr_ts_logger.py, frts.py and check_emu_capture.py from THIS zip.

Files
-----
  pico_flexray_dde_emu_v1z1_zv_ignon_v2.uf2       V2 board. Starts nothing by itself: the
                                                 laptop logger switches the FlexRay emulator
                                                 and the private CAN frames on (as v1h.3 ignon).
  pico_flexray_dde_emu_v1z1_zv_standalone_v2.uf2  V2 board. Emulates from power-up with no
                                                 laptop (as v1h.3 standalone: = --emulate
                                                 --can-tx --fake-start cas --start-profile short).
  fr_ts_logger.py, frts.py   laptop logger (decodes the private frames, warns if the Pico
                             ever sends a BMW ID)
  check_emu_capture.py, dde_emu_model.py   capture checker (ZV mode found automatically)
  zv_link/pico_zv_private_can.md + .dbc    the private frame spec (+ v1z.1 bit 31)
  SHA256SUMS
There is no V1-board build: the ZV link needs the V2 board's CAN chip. The v1h.3 images
are unchanged (built again from this tree: byte-identical to the v1h.3 zip).

What the Pico does in ZV mode
-----------------------------
  - FlexRay: as v1h.3. The engine / gear part of 0xE6, 0x28 (rpm, torque, gear echo),
    0x29, 0x3D, 0xFB... is taken from, in this order:
      1. PRIVATE  the ZV's private frames 0x304-0x307 (not sent by PR #276 yet, so this
                  never happens with today's ZV code);
      2. PT-CAN   the ZV's BMW frames: state from 0x3F9, rpm/torque from 0x0A5, gear from
                  0x3F9 + 0x0DC, MIL from 0x592 (0x3FD / 0x39A for the status page);
      3. OWN      the Pico's own v1h.3 story (ST_KL from 0x12F/0x74, brake, lever).
    "ZV there" = a ZV 0x0A5 in the last 100 ms (it is sent every 20 ms) and 0x3F9 in the
    last 2.5 s. A switch from OWN to the ZV waits until both say the same thing (both
    off, or both running), so FlexRay never jumps between "running" and "off".
  - If the ZV disappears while it said cranking / running / stopping: the Pico plays its
    own normal stop on FlexRay (82 -> 92 -> rpm run-down -> 80), puts the gear back to P,
    and does NOT restart the engine story by itself until the ZV is back saying "off".
    If no ZV is ever seen after the FlexRay wake, the Pico runs exactly the v1h.3 story.
  - PT-CAN TX (only with --can-tx, or the standalone image): 0x308 status (20 ms + on
    every FlexRay ST_KL change), 0x309 DSC (10 ms), 0x30A brake (20 ms), 0x30B cruise
    (50 ms), 0x30C FlexRay echo (100 ms + after every 0xE6 sent). CRC8 + alive counter
    as in the spec. Only while the car's own frames (0x0EF/0x12F/0x173) are less than
    200 ms old; first frames 20 ms after that.
  - Duplicate-ID check: the ZV's BMW frames and 0x304-0x307 are expected and never a
    fault. Anybody else sending 0x308-0x30F latches a fault: private TX stops, CAN chip
    back to Listen Only, the logger prints "CANTX FAULT: foreign sender of a private Pico
    ID". Cleared by the logger switching CAN TX off (Ctrl-C / restart) or a power cycle.
  - Lag (host co-sim, worst case over every position in the FlexRay cycle):
      0x3F9 change -> 0xE6 on FlexRay        <= 13.7 ms (2.7 FlexRay cycles), mean ~9-11 ms
      0x0A5 rpm    -> 0x28 rpm frame         <= 17.6 ms (0x28 rpm only goes out every 20 ms)
      0x0DC gear   -> 0x28 gear echo         <= 21.9 ms (same 20 ms frame)
    0xE6 is only sent in even cycles (10 ms) and each frame is built at the start of its
    cycle, so ~2-3 cycles is the floor without breaking the v1h.3 rules.

Wiring (same as the v1h.3 V2 board)
-----------------------------------
  - FlexRay as for v1h.3 (the real DDE/EGS FlexRay pair goes to the Pico, not the car).
  - Pico V2 CAN (MCP251863) on PT-CAN, as for v1h.3 V2 (CAN-H, CAN-L, ground).
  - The ZombieVerter's CAN port on the SAME PT-CAN (the bus the real DDE was on).
  - The real DDE and EGS must NOT be on PT-CAN (the ZV would see them as a second sender
    and go quiet with F11Fault = 1).
  - Only ONE Pico on the bus (a second one would trip the private-ID fault).
  - USB from the Pico to the laptop for the ignon image (and for logging with either).

ZombieVerter settings (in its web interface)
--------------------------------------------
  - Vehicle: BMW_F11 (PR #276 build).
  - Transmission: MUST MATCH THE CAR. 1 = Auto for the automatic (EGS) car, 0 = Manual
    for a manual box. The ZV's 0x3F9 / 0x0DC / 0x3FD gear content depends on it, and the
    Pico copies whatever the ZV sends onto FlexRay. Note for Manual: the ZV sends gear
    nibble 1 and no P lock in 0x3F9 even before the start; v1h.3 showed fixed P instead.
    The FlexRay side now shows the ZV's version. Not seen on a car yet.
  - Watch the ZV value F11Fault: 0 = ok, 1 = a second sender of a ZV frame (it stops
    sending), 2 = start fault. It must stay 0 for the whole test.

How to run (car)
----------------
Flash pico_flexray_dde_emu_v1z1_zv_ignon_v2.uf2 (hold BOOTSEL, copy the file). Then:

  Step A, Pico listens only (no Pico CAN TX at all):
  python3 fr_ts_logger.py -o ~/Downloads/FlexRay/fr-ts/f11_emu12_v1z1.csv --emulate --can --fake-start cas --start-profile short

  Step B, with the private frames 0x308-0x30C:
  python3 fr_ts_logger.py -o ~/Downloads/FlexRay/fr-ts/f11_emu13_v1z1.csv --emulate --can --can-tx --fake-start cas --start-profile short

  Check afterwards (either run):
  python3 check_emu_capture.py ~/Downloads/FlexRay/fr-ts/f11_emu12_v1z1.csv --engine --gear --flicker

--fake-start cas only arms the Pico's own fallback story; with the ZV there, the ZV's start
is what goes on FlexRay. The logger's "zv link:" status line shows the private frame counts
and the last PICO_Status (mode=RUN mirror_applied mirror_src_ptcan = mirroring the ZV).

Car-test checklist (stop at the first NO)
-----------------------------------------
  [ ] ZV Transmission set to match the car; Vehicle BMW_F11; F11Fault 0 before KL15.
  [ ] Real DDE/EGS off PT-CAN; one Pico; logger and firmware both say v1z.1
      ("firmware version: dde-emu-zv-v1z.1", no VERSION MISMATCH box).
  [ ] Step A first. KL15 on: no new fault lamps compared to v1h.3; F11Fault stays 0;
      the logger shows NO "own-tx" frames (the Pico must be silent without --can-tx).
  [ ] START with the brake: the ZV runs its start; FlexRay 0xE6 goes 81 -> 82 and CAS
      releases the starter as in run 10 (check_emu_capture: "0x82 sent before CAS
      released", "0xE6 follows 0x3F9 ... [need <= 25 ms]").
  [ ] Auto car: P -> D -> N -> R -> P with the brake; dash gear and FlexRay follow the ZV.
      Manual car: no P on the dash is expected (see the Manual note above).
  [ ] Stop (START again / KL15 off): 0xE6 92 -> 80, no restart, F11Fault 0.
  [ ] Step B: the same again with --can-tx. Logger: "PRIVATE ZV-LINK TX ENABLED", the
      "zv link:" line counts 308/309/30A/30B/30C going up, no "WARNING: ... sent BMW IDs",
      no CANTX FAULT. check_emu_capture.py section 1z all PASS ("Pico BMW IDs sent: 0").
      F11Fault stays 0 (the ZV must not see a second sender).
  [ ] Fail-safe (optional, engine "running", car stationary, foot on the brake): unplug
      the ZV's CAN. FlexRay must play 92 -> 80 and P within about 1.3 s, and stay off.
      Plug back in, KL15 off/on: normal again.
  [ ] Sleep: KL15 off, lock, bus quiet after ~1 min as with v1h.3.
Only after all of this passed on the car may the build be published.

What was tested (host only)
---------------------------
  - tests/host/run.sh: everything from v1h.3 still passes; v1h.3 images byte-identical.
  - test_zv_link: private CRC/alive, private TX rates and gate, the mirror precedence,
    the fail-safe (ZV lost -> own stop at +90 ms, 80 at +1285 ms, P, no restart), the own
    story unchanged with no ZV.
  - test_ptcan_tef "zv": the real ptcan_emu.c only sends 0x308-0x30C; the ZV's BMW frames
    on the bus do not trip the duplicate check; a foreign 0x309 stops TX at once (0
    frames after it); "CAN TX off" from the host clears it.
  - Co-simulation: the ZV PR #276 BMW_F11 code and the Pico's PT-CAN side on one pretend
    bus, automatic and manual, 25 s each: 0 Pico BMW IDs, F11Fault 0 throughout, the
    private frames at their rates; a foreign 0x308 latches at once; ZV lost mid-run.
  - FlexRay replay on car run 2 with the co-sim's PT-CAN traffic: 0xE6 never in an odd
    cycle, E2E all good, every 0x3F9 change shown on 0xE6 (lag above), start/stop at the
    same FlexRay cycle as v1h.3 except: the first 0x80 10 ms earlier, and at the end of
    the recording (ZV goes silent) 0x92 / 0x80 260 ms earlier, because a lost ZV is
    seen 100 ms after its last 0x0A5.
  - check_emu_capture.py ZV mode on the co-sim capture: all ZV-mode checks pass.
```
