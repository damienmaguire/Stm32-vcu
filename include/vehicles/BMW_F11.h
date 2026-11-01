/*
 * This file is part of the stm32-vcu project.
 *
 * Copyright (C) 2026 Damien Maguire
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BMW_F11_h
#define BMW_F11_h

// BMW F11 (5 Series Touring, 2010-2017) with the diesel engine and the
// ZF 8-speed gearbox removed.
//
// The DDE was the diesel engine ECU. The dash, DSC (stability control),
// steering and heater still expect it. This class takes the DDE's place
// on PT-CAN, the powertrain CAN bus at 500 kbit/s. Plug that bus into
// whichever ZombieVerter port is selected as "Vehicle CAN".
//
// We listen on PT-CAN for ignition and the brake pedal, and we send the
// engine frames (rpm, temperatures, "engine running", check-engine light
// off). We send nothing on PT-CAN2. That second bus belonged to the
// gearbox.
//
// FlexRay is the car's other fast network. A separate Pico board
// emulates the DDE there. This class does not speak FlexRay and does
// not send anything to the Pico.
//
// Words used below:
//   KL15  ignition on
//   KL50  start / crank
//   MIL   the check-engine light
//   E2E   the checksum on important frames. Byte 0 is a CRC and the low
//         half of byte 1 is a counter that steps 0..14. If either is
//         wrong the receiving ECU throws the frame away.
//
// Any F-series (F10/F11/F30/F31...) with its engine removed can use this
// class. The Transmission setting says what gearbox the car had:
//   Manual  only the engine frames are sent. The gear fields in 0x3F9
//           and 0x0DC carry fixed "manual / neutral" values.
//   Auto    the class also stands in for the removed EGS (the gearbox
//           ECU): gear display 0x3FD, gear state 0x39A and NM 0x518,
//           with the lever (0x197) and brake gear logic (dde_gear.c).
//           The VCU only gets R or D once that gear is engaged.
//
// Timing and bytes copy the Pico emulator v1h.3 (pico-flexray
// src/ptcan_emu_core.c, dde_engine.c, dde_gear.c) and the spec
// flexray-re-notes analysis/pico_zv_private_can.md section 5. The Pico
// keeps FlexRay. Each PT-CAN id must be sent by one box only.
//
// When we may transmit (the v1h.3 bus gate):
//   Nothing either while KL15 is off, no other ECU sent an NM frame
//   (0x500-0x57F) for 1.95 s and our own NM (0x512 / 0x518) has ended
//   ("prepare sleep"), so we never keep the car awake.
//   W (wake) is the first 0x12F, 0x173 or 0x0EF after at least 1 s of
//   silence. Every frame starts at its own captured time after W, and
//   all counters restart. 200 ms with none of those three ids stops all
//   transmission at once, so a silent car is never woken by us.
//   If one of our ids (0x3F9, 0x0A5, 0x512; automatic also 0x3FD,
//   0x518) arrives from another box, we stop for good (F11Fault = 1)
//   until a parameter is written or the VCU restarts.
//
// "Engine running" on the bus follows a short fake engine start, as the
// Pico v1h.3 SHORT profile: ST_KL D (crank) with the brake down starts
// it, and 0x3F9 shows 0x82 (running) about 172 ms later. Motor torque is
// only allowed while that story says running AND the VCU is in RUN. If
// the VCU has not reached RUN 5 s after 0x82, the story stops
// (F11Fault = 2) until the next crank request. 0x592 stays "MIL off" as
// in v1h.3: the "lamp on" byte was never proven on this car.
//
#include "vehicles/vehicle.h"
#include <stdint.h>

class BMW_F11 : public Vehicle {
public:
  BMW_F11();
  // Choose the Vehicle CAN port and ask to be told about 0x12F and 0x173.
  void SetCanInterface(CanHardware *c);
  // 10 ms tick: the 20 ms engine frames and 0x08F (30 ms).
  void Task10Ms();
  // 100 ms tick: fuel counter and 12 V.
  void Task100Ms();
  // 1 ms tick: bus gate, fake engine story, gear logic, and the frames
  // whose timing must be exact (0x3F9, 0x592; automatic: 0x3FD, 0x39A,
  // 0x518).
  void Task1Ms();
  // 200 ms tick: the 1 s constant frames 0x396 and 0x1B9.
  void Task200Ms();
  // Gear for the VCU. NEUTRAL (no torque) unless our engine story says
  // running. Manual: then the normal direction inputs decide. Automatic:
  // R or D only once that gear is engaged.
  bool GetGear(Vehicle::gear &outGear);
  // A parameter was written: forget the "second sender" fault.
  void ClearFaults() { foreignTx = false; }
  // Inverter rpm. While driving, the dash is shown at least 750 rpm.
  void SetRevCounter(int s) { motorRpm = s; }
  // Inverter coolant temperature, degrees C. Shown on 0x3F9.
  void SetTemperatureGauge(float temp) { coolantC = temp; }
  // Incoming PT-CAN: ignition (0x12F), brake pedal (0x173) and, for an
  // automatic, the gear lever (0x197).
  void DecodeCAN(int id, uint32_t *data);
  // True when KL15 (ignition) is on. The rest of the VCU treats this
  // the way an E46 treats the terminal-15 pin.
  bool Ready();
  // True when the brake is down and start is requested (0x12F crank
  // position, or the wired start button). That is what lets the VCU
  // leave OFF and begin precharge.
  bool Start();

  // Driver demand in Nm, limited by the DSC torque ceiling. With no
  // Pico frame the ceiling is "no limit" and driverNm comes back as-is.
  float FinalTorqueNm(float driverNm) const;

  // What one gear puts in the frames: 0x3FD byte 2, 0x39A bytes 3-4,
  // 0x3F9 P lock bits and gear nibble, 0x0DC bytes 2 and 5.
  struct GearBytes {
    uint8_t b3fd, a3, a4, park, nib, dc2, dc5;
  };

private:
  // 0x0A5, 0x0D9, 0x0DC and 0x0F3, every 20 ms.
  void SendFastFrames();
  // 0x3F9 temperatures and engine-running, every 1 s and on change.
  void Send3F9(bool ready);
  // 0x08F wheel torque / engine state, every 30 ms.
  void Send08F();
  // 0x2C4 fuel used and 0x281 12 V, every 100 ms.
  void Send100MsFrames();
  // Notice PT-CAN waking or going to sleep.
  void NoteBusState();
  void OnBusWake();
  void OnBusSleep();
  // True while we may transmit: a 0x12F, 0x173 or 0x0EF in the last
  // 200 ms, and no second sender seen (v1h.3 gate).
  bool BusAwake() const;
  // Note one PT-CAN frame from the car. Starts a new wake after >= 1 s
  // of silence.
  void NoteRx();
  // ms since the wake (W), since KL15 on (ON) and since KL15 off (OFF).
  uint32_t SinceWake() const { return nowMs - wakeMs; }
  uint32_t SinceOn() const { return nowMs - onMs; }
  uint32_t SinceOff() const { return nowMs - offMs; }
  bool AutoBox() const { return Param::GetInt(Param::Transmission) == 1; }
  // Fake engine start/stop (v1h.3 dde_engine.c, SHORT profile only).
  void EngineStep();
  void EngineCheckStart();
  void EngineCheckStop();
  void EngineStopNow();
  void EngineOutputs(uint32_t dtMs);
  void EngineWatchdog();
  bool BrakeRecent(uint32_t ms) const;
  // 0x592 MIL-off frames on the v1h.3 timeline (ON+98 ms, every 5.1 s,
  // one last frame at OFF+54 ms).
  void Send592();
  // rpm for the dash. At least 750 while the inverter is running.
  uint16_t RpmTx() const;
  // Driver torque in Nm. Currently 0 until a real Nm source exists.
  float DriverDemandNm() const;
  // Drop a DSC torque cut that has not been refreshed for 500 ms.
  void AgeDscLimit();
  // Decode ignition and start from 0x12F.
  void Handle12F(uint8_t *bytes);
  // Decode the brake pedal from 0x173.
  void Handle173(uint8_t *bytes);
  // Decode total brake torque from 0x0EF.
  void Handle0EF(uint8_t *bytes);
  // OR the CAN brake pedal into the normal brake input.
  void ApplyBrake();
  // Optional brake-torque regen. Does nothing while BrkRegen is off.
  void UpdateBrakeRegen();
  // True while the engine story says running (0x82 on 0x3F9).
  bool EngineRunning() const;
  // Automatic gearbox (v1h.3 dde_gear.c), stepped every 1 ms.
  void GearTask();
  void GearLever(const uint8_t *d, uint32_t ms);
  void GearAct(uint8_t dir, uint8_t depth, uint32_t ms);
  void GearStart(uint8_t to, uint32_t ms);
  void GearStep();
  bool GearBrake(uint32_t ms) const;
  // The gear bytes the EGS frames carry (see GearOut). False = fixed P
  // (no change to the templates), as v1h.3 while P is locked or on a
  // manual box.
  bool GearOut(GearBytes &g) const;
  // EGS frames (automatic only): 0x3FD, 0x39A and NM 0x518.
  void Send3FD();
  void Send39A();
  void Send518();
  // Slow DDE frames (v1h.3 step tables): 0x381, 0x3B3, 0x397, 0x3FB
  // and NM 0x512.
  void SendSlowDde();
  void Send512();
  // NM frames allowed now (v1h.3 frame_active)? And the "prepare
  // sleep" state: KL15 off, no other NM for 1.95 s, ours released.
  bool Nm512Active() const;
  bool Nm518Active() const;
  bool PrepSleep() const;
  uint32_t NmCeilingRef() const;
  // One frame's v1h.3 send timing (pte_next / pte_commit): the first
  // frame at W + firstMs, then a fixed grid. With gapMs > 0 a changed
  // payload (from byte `from`) also goes out at once, at most every
  // gapMs, without moving the grid.
  struct TxSlot {
    uint32_t next, last;
    bool started, pend, cyc;
    uint8_t want[8];
  };
  bool SlotDue(TxSlot &s, const uint8_t *d, uint8_t from, uint8_t dlc,
               uint32_t firstMs, uint32_t gapMs);
  void SlotSent(TxSlot &s, const uint8_t *d, uint32_t periodMs);
  // TODO: Pico frame that carries the DSC torque ceiling. No id yet.
  void HandleDscTorqueCeiling(uint32_t *data);

  static uint8_t Crc8_1d(const uint8_t *data, int len, uint8_t crc);
  static void E2ESeal(uint8_t *frame, int dlc, uint16_t dataId, uint8_t *alive);

  int motorRpm;
  float coolantC;
  // Our own clock, +1 every Task1Ms. All times below are in ms.
  volatile uint32_t nowMs;
  uint32_t lastRxMs; // last 0x12F, 0x173 or 0x0EF
  uint32_t wakeMs;   // W
  volatile uint16_t wakeSeq;
  uint16_t wakeSeqSeen;
  bool rxEver;
  bool busWasAwake;
  bool foreignTx; // another box sends 0x3F9 / 0x0A5: we stay silent
  // KL15 history in this wake, for the frame timelines.
  bool kl15Ever;
  uint32_t onMs, offMs;
  // Last 0x12F that said KL15 on (or W). NM frames stop 60 s after it.
  uint32_t lastOnEvMs;
  // Last NM frame (0x500-0x57F) from another ECU, or W.
  volatile uint32_t lastFnmMs;
  // 0x592: next frame time, and the one last frame after KL15 off.
  uint32_t next592;
  bool final592;
  // The start watchdog fired (VCU not in RUN 5 s after we said
  // "running"). Shown as F11Fault = 2; cleared when KL15 goes off.
  bool startFault;
  // Raw ST_KL and the last time the brake was seen down (0x173 both
  // switches, or the 0x12F byte 6 mirror). For the engine story.
  uint8_t stKl;
  uint32_t stKlMs;
  bool stKlHave;
  uint32_t brakeDownMs;
  bool brakeDownHave;
  // The fake engine (v1h.3 dde_engine.c eng_t, cut down to what PT-CAN
  // needs). Phases: 0 off, 1 crank, 2 run, 3 stop, 4 stopped.
  struct EngStory {
    uint8_t phase;
    uint32_t t0, t82, ts, t80, trpm, t9, lastStep;
    bool curRun, hasRun, startD, dEnded, t82Have, t9Have, stepHave;
    bool wdOk;    // the VCU reached RUN after this start
    bool wdBlock; // watchdog fired: no new start until ST_KL leaves D
    uint16_t rpmAtStop;
    uint32_t f3q8, fuelQ10;
    // outputs (v1h.3 eng_out_t)
    bool active;
    uint8_t cntHi, e6b3, e6b4, e6ready;
    uint16_t rpm4, torque, f3val, fuel;
    uint8_t r29b5;
    bool f97c;
    // slow frames: 0x3FB "injecting", 0x381 running, 0x3B3 bytes 1-3,
    // 0x397 bytes 0-2, and the end of the 0x397 stop walk
    bool inj, r381Run, fb9Set, r397Set, stopWalkDone;
    uint8_t fb9[3], r397[3];
  } eng;
  // 1 s frames on the 200 ms task: ticks since W.
  uint8_t tick200;
  uint16_t tick10;
  // The 20 ms group and 0x08F start on their first-frame time after W
  // and then keep their own grid on the 10 ms task.
  uint16_t fastTick;
  bool fastOn;
  uint16_t tick8F;
  bool on8F;
  uint8_t last8F7;

  uint8_t aliveA5;
  uint8_t aliveD9;
  uint8_t aliveDc;
  uint8_t aliveF3;
  uint8_t alive8F;
  uint8_t alive3F9;

  // Gear state (v1h.3 gear_t). Gears 0 P, 1 R, 2 N, 3 D. Stages:
  // 0 P locked, 1 leaving P, 2 steady, 3 going to P, 4 P not yet
  // locked, 5 P released by a lever push (brake on).
  struct GearState {
    uint8_t target, from, stage, relPrev;
    uint32_t tStage, tAct;
    bool lvHave;
    uint32_t lvMs;
    uint8_t gestDir, gestDepth, pendDir, pendDepth;
    uint32_t pendMs, pushMs;
    bool pushRel, pbtnDone, running;
  } gear;
  // 0x197 from DecodeCAN for the 1 ms task. seq is odd while writing.
  volatile uint8_t lv197[4];
  volatile uint32_t lv197Ms;
  volatile uint8_t lv197Seq;
  uint8_t lv197Seen;
  // Last 0x173 with both brake switches on. The gear logic uses only
  // this (dde_emu.c brmb), not the 0x12F mirror.
  volatile uint32_t brake173Ms;
  volatile bool brake173Have;
  uint8_t alive3FD;
  TxSlot s3FD, s39A, s518;
  TxSlot s381, s3B3, s397, s3FB;
  // 0x512: next grid time (set at KL15 on), and how many of the four
  // pre-KL15 frames went out in this wake.
  uint32_t next512;
  uint8_t pre512;

  bool seen3f9;
  uint32_t next3f9, last3f9Ms;
  uint8_t last3f9[6];

  // From 0x12F byte 2, low nibble. 6 = off, 9 = accessory (holds KL15
  // only if it was already on), >= 0xA = ignition on, >= 0xD = crank.
  // 0xE/0xF leave these alone. See Handle12F.
  bool kl15On;
  bool kl50On;

  // From 0x173 byte 7. Both switch bits must be set for "pedal down".
  // One bit alone is a plausibility warning, not a press. Which bit
  // is the brake-light switch is still unconfirmed.
  bool brakePressed;
  bool brakeOneSwitch;
  bool brakePressure; // bit 2, pressure above about 5 bar
  bool brakeSeen;
  // 10 ms ticks since the last 0x173. Past 50 (500 ms) we let go.
  uint8_t brakeAge10ms;
  // 0x0EF bytes 2-3, little-endian, Nm = raw - 32000. Negative while
  // braking. 0 when the frame has gone quiet.
  float brakeTorqueNm;
  bool brakeTorqueSeen;
  uint8_t brakeTorqueAge10ms;

  // DSC torque ceiling. The DSC talks to the DDE on FlexRay only. The
  // pico will forward that limit on CAN. The ID and the byte layout are
  // not defined, so dscTorqueCanId stays 0 and no filter is registered.
  bool dscLimitFresh;
  uint8_t dscLimitAge10ms;
  float dscLimitNm;
  uint32_t dscTorqueCanId;
};

#endif /* BMW_F11_h */
