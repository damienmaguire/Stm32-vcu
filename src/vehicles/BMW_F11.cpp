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

#include "vehicles/BMW_F11.h"
#include "stm32_can.h"
#include "utils.h"

// What this file does is described at the top of BMW_F11.h.
// Short version: on PT-CAN we imitate the removed diesel ECU (DDE).
// FlexRay is the Pico's job. PT-CAN2 (the old gearbox bus) is silent.
//
// Every protected frame uses E2E. That is the car's anti-corruption
// check: byte 0 is a CRC-8 (poly 0x1D, start at 0), computed over the
// frame's data id (low byte, then high byte) and then bytes 1..end.
// The low half of byte 1 is an alive counter, 0..14, skipping 15.
// The first frame after the bus wakes sends 1. A bad CRC or a skipped
// count makes the ECU ignore the frame.

// CRC-8 used by E2ESeal. One routine for every protected frame.
uint8_t BMW_F11::Crc8_1d(const uint8_t *data, int len, uint8_t crc) {
  while (len--) {
    crc ^= *data++;
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80)
        crc = (uint8_t)((crc << 1) ^ 0x1D);
      else
        crc = (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// Stamp the alive counter and the CRC onto one frame.
// alive is per CAN id. It starts at 0 so the first call sends 1.
void BMW_F11::E2ESeal(uint8_t *frame, int dlc, uint16_t dataId,
                      uint8_t *alive) {
  if (*alive >= 14)
    *alive = 0;
  else
    *alive = (uint8_t)(*alive + 1);

  frame[1] = (uint8_t)((frame[1] & 0xF0) | *alive);

  uint8_t id[2];
  id[0] = (uint8_t)(dataId & 0xFF);
  id[1] = (uint8_t)(dataId >> 8);
  frame[0] = Crc8_1d(frame + 1, dlc - 1, Crc8_1d(id, 2, 0));
}

// Crank torque for 0xA5. The car uses
//   raw = (Nm + 1023.5) * 2
// so 0 Nm is 2047 (0x7FF). 4095 means "no value", so we stop at 4094.
static uint16_t TorqueNmToRaw(float nm) {
  float raw = (nm + 1023.5f) * 2.0f;
  if (raw < 0.0f)
    raw = 0.0f;
  if (raw > 4094.0f)
    raw = 4094.0f;
  return (uint16_t)(raw + 0.5f);
}

// Coolant and oil on 0x3F9. The car stores degrees C plus 48
// (0 reads as -48 C). 255 means "no value", so we stop at 254.
static uint8_t TempPlus48(float degC) {
  float raw = degC + 48.0f;
  if (raw < 0.0f)
    raw = 0.0f;
  if (raw > 254.0f)
    raw = 254.0f;
  return (uint8_t)(raw + 0.5f);
}

// Gear codes the dash and the gearbox-less chassis still expect.
// 0xDC byte 2, and the low half of 0x3F9 byte 6:
//   Park 0x22 / nibble 3,  Reverse 0x25 / nibble 2,  Drive 0x24 / nibble 5.
// Neutral was never captured. Dir 0 is what the VCU writes for
// park/neutral when it shuts down, so that position uses the Park code.
struct GearCode {
  uint8_t dc;
  uint8_t nibble;
};

// Turn the VCU direction (-1 reverse, 0 neutral, 1 drive, 2 park)
// into the two gear codes above.
static GearCode GearFromDir(int dir) {
  GearCode gear;
  switch (dir) {
  case -1: // reverse
    gear.dc = 0x25;
    gear.nibble = 2;
    break;
  case 1: // drive
    gear.dc = 0x24;
    gear.nibble = 5;
    break;
  case 2: // park
  case 0: // neutral, see above
  default:
    gear.dc = 0x22;
    gear.nibble = 3;
    break;
  }
  return gear;
}

// Accelerator position for 0xD9. 0.025 % per bit, so
// raw = percent / 0.025 = percent * 40, and 100 % is 4000.
// Both pedal slots get the same number. Regen (a negative throttle)
// is a released pedal, so the raw value is 0.
static uint16_t PedalRaw() {
  int percent = Param::GetInt(Param::potnom);
  if (percent < 0)
    percent = 0;
  if (percent > 100)
    percent = 100;
  return (uint16_t)(percent * 40);
}

// Start asleep: busQuiet100 = 20 means "2 seconds of silence already",
// so we do not transmit until the car speaks first.
BMW_F11::BMW_F11()
    : motorRpm(0), coolantC(0), busQuiet100(20), busWasAwake(false), tick10(0),
      aliveA5(0), aliveD9(0), aliveDc(0), aliveF3(0), alive8F(0), alive3F9(0),
      seen3f9(false), age3f910(0), milOffSent(false), kl15On(false),
      kl50On(false), brakePressed(false), brakeOneSwitch(false),
      brakePressure(false), brakeSeen(false), brakeAge10ms(0), brakeTorqueNm(0),
      brakeTorqueSeen(false), brakeTorqueAge10ms(0), startSeen(false),
      dscLimitFresh(false), dscLimitAge10ms(0), dscLimitNm(0),
      dscTorqueCanId(0) {
  for (int i = 0; i < 6; i++)
    last3f9[i] = 0;
}

// Tell the CAN driver which port is PT-CAN, and which incoming ids
// we care about. We do not listen to every frame: the driver cannot
// tell the vehicle port from the inverter port, and counting inverter
// traffic would make us the box that wakes the car.
void BMW_F11::SetCanInterface(CanHardware *c) {
  can = c;

  // 0x12F  ignition and start, from the body gateway (a copy of
  // FlexRay 0x74). Normally every 100 ms. While the terminal state
  // is C or D (start in progress / crank) the gateway sends it every
  // 20 ms. We only receive it. See Handle12F.
  can->RegisterUserMessage(0x12F);
  // 0x173  every 100 ms. Brake switches, a copy of the DSC FlexRay
  // frame. See Handle173.
  can->RegisterUserMessage(0x173);
  // 0x0EF  every 20 ms. Total brake torque. See Handle0EF.
  can->RegisterUserMessage(0x0EF);

  // TODO: DSC torque cuts arrive on FlexRay, not here. The Pico will
  // forward them later. No CAN id is assigned. See
  // HandleDscTorqueCeiling() and dde_dsc_torque_rx.md.
  // TODO: cruise torque is FlexRay 0x21 (1 Nm per bit, offset -32000).
  // The stalk is not on PT-CAN. No CAN id is assigned. See
  // dde_cruise_rx.md. Do not invent a 0x12F replacement if the
  // gateway is removed; something else must send the real frame.
}

// Ignition present? The E46 answers this from the terminal-15 pin.
// The F11 answers it from 0x12F (KL15). The pin is left out because
// it floats, and the bus is what the diesel ECU trusted.
bool BMW_F11::Ready() { return kl15On; }

// Start request, and only with the brake pedal down.
// 0x12F crank (KL50, nibble D, held through C) is the car's START
// button. The CAS does not send a separate start frame, and it only
// reaches D if its own brake check passed. We still require both
// brake switches here, so a single-switch fault cannot start the car.
// The wired start button is accepted the same way.
bool BMW_F11::Start() {
  if (!Param::GetBool(Param::din_brake))
    return false;
  return kl50On || Param::GetBool(Param::din_start);
}

// PT-CAN just woke. Restart the alive counters so the next frame
// carries 1, and send 0x3F9 straight away rather than waiting a second.
void BMW_F11::OnBusWake() {
  busWasAwake = true;
  tick10 = 0;
  aliveA5 = 0;
  aliveD9 = 0;
  aliveDc = 0;
  aliveF3 = 0;
  alive8F = 0;
  alive3F9 = 0;
  seen3f9 = false;
  age3f910 = 0;
}

// Two seconds with no 0x12F, 0x173 or 0x0EF. Stop talking, and forget
// ignition and the brake so we cannot hold the car awake.
void BMW_F11::OnBusSleep() {
  busWasAwake = false;
  aliveA5 = 0;
  aliveD9 = 0;
  aliveDc = 0;
  aliveF3 = 0;
  alive8F = 0;
  alive3F9 = 0;
  seen3f9 = false;
  // The rundown normally sends nibble 6 before the bus goes quiet.
  // If the frames just stop, drop ignition and the CAN brake with them.
  kl15On = false;
  kl50On = false;
  brakePressed = false;
  brakeOneSwitch = false;
  brakePressure = false;
  brakeSeen = false;
  brakeTorqueNm = 0;
  brakeTorqueSeen = false;
  startSeen = false;
}

// Called every 10 ms and every 100 ms. Notices the bus going to
// sleep or waking, and runs OnBusSleep / OnBusWake once per change.
void BMW_F11::NoteBusState() {
  if (!BusAwake()) {
    if (busWasAwake)
      OnBusSleep();
    return;
  }
  if (!busWasAwake)
    OnBusWake();
}

// A frame arrived. Only 0x12F and 0x173 prove that PT-CAN is up.
// Other ids (inverter, charger) are ignored for that test.
void BMW_F11::DecodeCAN(int id, uint32_t *data) {
  uint8_t *bytes = (uint8_t *)data;

  // These three ids are PT-CAN from the car. Inverter and charger
  // frames must not count, or we could be the box that wakes the bus.
  if (id == 0x12F || id == 0x173 || id == 0x0EF)
    busQuiet100 = 0;

  switch (id) {
  case 0x12F:
    Handle12F(bytes);
    break;
  case 0x173:
    Handle173(bytes);
    break;
  case 0x0EF:
    Handle0EF(bytes);
    break;
  default:
    break;
  }

  // DSC ceiling. dscTorqueCanId is 0 until the pico frame is defined.
  if (dscTorqueCanId != 0 && id == (int)dscTorqueCanId)
    HandleDscTorqueCeiling(data);
}

// 0x12F  PT-CAN  ignition and start, from the body gateway.
// Normally every 100 ms. While the nibble below is C or D the gateway
// sends it every 20 ms (not "100 ms plus an occasional extra").
// The CAS (key and start module) does not send a separate start frame.
// Crank is this nibble. Tests reached "engine running" with no
// immobiliser exchange, so we send none. If the CAS or the gateway
// is removed, something else has to transmit 0x12F or ignition stays
// off and we stop transmitting 2 seconds later.
//
//   byte 2, low nibble (bits 16-19), called ST_KL:
//     6      ignition off. The bus can still be awake.
//     9      accessory. Does not turn KL15 on by itself. If KL15 is
//            already on, a 9 holds it on (the diesel ECU did this).
//     A, B   KL15, ignition on
//     C      start in progress. Holds KL50 only if crank was already on.
//     D      KL50, the start request / crank
//     E, F   invalid. Keep the last ignition and start state.
//   byte 6   brake-switch mirror. Released 0x30, both switches 0x33.
//            The low two bits copy 0x173 bits 0 and 1. We do not use
//            this byte as the brake input; 0x173 is that input.
//   Other bytes are not used.
void BMW_F11::Handle12F(uint8_t *bytes) {
  uint8_t st = bytes[2] & 0x0F;
  if (st >= 0x0E)
    return;

  if (st >= 0x0A || (st == 0x09 && kl15On))
    kl15On = true;
  else
    kl15On = false;

  if (st >= 0x0D || (st == 0x0C && kl50On))
    kl50On = true;
  else
    kl50On = false;
}

// 0x173  PT-CAN  every 100 ms  brake switches.
// Gateway copy of the DSC (stability control) FlexRay frame 0x2F.
// The diesel ECU had no brake-switch wire. It read this instead.
//
//   byte 7:
//     bit 0  one brake-switch channel
//     bit 1  the other channel
//     bit 2  brake pressure above about 5 bar (about 450 Nm on 0x0EF)
//     bit 4  car standing still. Not the pedal.
//     bit 5  value not valid (seen for a moment after wake). We then
//            keep the previous pedal state.
//     bits 6-7  normally set. Ignored.
// Pedal down means BOTH bit 0 and bit 1 are set. That is what enables
// start, and what the rest of the VCU treats as the brake pedal for
// creep and regen. Exactly one bit is a plausibility warning (BrkWarn
// on the web interface), not a press. Which bit is the brake-light
// switch is still not confirmed. A slow press on the car has to
// settle that.
void BMW_F11::Handle173(uint8_t *bytes) {
  brakeSeen = true;
  brakeAge10ms = 0;

  // Bit 5 = invalid / init. Keep the last pedal state.
  if (bytes[7] & 0x20)
    return;

  uint8_t sw = bytes[7] & 0x03;
  if (sw == 0x03) {
    brakePressed = true;
    brakeOneSwitch = false;
  } else if (sw == 0x00) {
    brakePressed = false;
    brakeOneSwitch = false;
  } else {
    brakePressed = false;
    brakeOneSwitch = true;
  }
  brakePressure = (bytes[7] & 0x04) != 0;
}

// 0x0EF  PT-CAN  every 20 ms  total brake torque.
// Gateway copy of FlexRay 0x2B. Little-endian: byte 2 is the low
// byte, byte 3 the high byte. Nm = raw - 32000, so 32000 is 0 Nm
// and the value goes negative as the driver brakes harder.
// Captured examples: about -1134 light, -2952 firm, -6959 full.
// Bytes 5-6 are a second copy within a few Nm. We publish byte 2-3
// as BrkTrq. Nothing is transmitted in reply.
void BMW_F11::Handle0EF(uint8_t *bytes) {
  uint16_t raw = (uint16_t)(bytes[2] | (bytes[3] << 8));
  brakeTorqueNm = (float)raw - 32000.0f;
  brakeTorqueSeen = true;
  brakeTorqueAge10ms = 0;
  Param::SetFloat(Param::BrkTrq, brakeTorqueNm);
}

// Put the CAN brake pedal into the normal brake input, after the
// wired pin has already been sampled. A pressed pin stays pressed.
// 50 ticks of 10 ms is 500 ms: if 0x173 disappears for that long we
// release the CAN pedal so a dead bus cannot leave it stuck on.
void BMW_F11::ApplyBrake() {
  if (brakeSeen) {
    if (brakeAge10ms < 255)
      brakeAge10ms++;
    // 0x173 is 100 ms. Half a second with no frame drops the CAN brake
    // so a dead bus cannot leave the pedal latched. The wired pin is
    // sampled on its own, in GetDigInputs.
    if (brakeAge10ms > 50) {
      brakePressed = false;
      brakeOneSwitch = false;
      brakePressure = false;
      brakeSeen = false;
    }
  }
  if (brakeTorqueSeen) {
    if (brakeTorqueAge10ms < 255)
      brakeTorqueAge10ms++;
    // 0x0EF is 20 ms. Same half-second quiet limit.
    if (brakeTorqueAge10ms > 50) {
      brakeTorqueNm = 0;
      brakeTorqueSeen = false;
      Param::SetFloat(Param::BrkTrq, 0);
    }
  }

  Param::SetInt(Param::BrkWarn, brakeOneSwitch ? 1 : 0);
  Param::SetInt(Param::BrkPress, brakePressure ? 1 : 0);

  // GetDigInputs has already stored the brake pin. OR the CAN pedal
  // in. Both switch bits are required, so one bit does not count.
  // Creep and regen in the throttle code read this same brake input.
  if (brakePressed)
    Param::SetInt(Param::din_brake, 1);
}

// BrkRegen is off by default. When it is turned on, brake torque from
// 0x0EF scales the normal brake-regen setting. BrkRegenNm is the
// torque (default 7000 Nm, a full press) that means 100 percent.
// A light press (~1100 Nm) is then a small fraction of brake regen.
// Coasting regen, with the pedal up, is unchanged.
void BMW_F11::UpdateBrakeRegen() {
  float pct = 0;
  if (Param::GetInt(Param::BrkRegen) != 0 && brakeTorqueSeen) {
    float full = Param::GetFloat(Param::BrkRegenNm);
    if (full < 1.0f)
      full = 1.0f;
    float mag = -brakeTorqueNm;
    if (mag > 0.0f)
      pct = (mag / full) * 100.0f;
    if (pct > 100.0f)
      pct = 100.0f;
  }
  Param::SetFloat(Param::BrkRegenPct, pct);
}

// The bus picture of a running engine. True only when the inverter
// is in RUN, and only after start was requested with the brake down.
// KL15 (ignition) alone stays "engine stopped", so we never tell the
// CAS the engine is running before START. The check-engine light goes
// off at the same moment.
bool BMW_F11::EngineRunning() const {
  return startSeen && Param::GetInt(Param::opmode) == MOD_RUN;
}

// TODO: DSC torque ceiling from the Pico. Not implemented.
// DSC (stability control) tells the diesel ECU to cut torque on
// FlexRay only (dde_dsc_torque_rx.md). The Pico will forward that
// limit later. Cruise is the same kind of gap: FlexRay 0x21 carries
// a requested wheel torque, 1 Nm per bit with 32000 meaning 0 Nm
// (dde_cruise_rx.md). Neither has a CAN id yet, so this function is
// never called and the ceiling stays "no limit". When a frame is
// defined, store dscLimitNm, set dscLimitFresh, and zero
// dscLimitAge10ms. Do not send immobiliser frames, PT-CAN2, 0x137
// or 0x6xx diagnostics from here.
void BMW_F11::HandleDscTorqueCeiling(uint32_t *data) { (void)data; }

// Forget a DSC torque cut that has gone quiet. 50 ticks of 10 ms is
// 500 ms. The Pico frame does not exist yet, so this does nothing
// until HandleDscTorqueCeiling is filled in. The 500 ms figure is a
// placeholder until that frame's real period is known.
void BMW_F11::AgeDscLimit() {
  if (!dscLimitFresh)
    return;
  if (dscLimitAge10ms < 255)
    dscLimitAge10ms++;
  if (dscLimitAge10ms > 50)
    dscLimitFresh = false;
}

// Torque the car is told we are making, in Nm. The smaller of the
// driver's demand and the DSC limit. No Pico frame, or one older than
// 500 ms, means there is no limit. The sign of a future cut (a cap,
// or a request to regen) is not known yet, so this is a plain minimum.
float BMW_F11::FinalTorqueNm(float driverNm) const {
  if (!dscLimitFresh)
    return driverNm;
  if (driverNm < dscLimitNm)
    return driverNm;
  return dscLimitNm;
}

// Driver demand in Nm. The "torque" value on the web interface is the
// raw number sent to the inverter (Leaf is about +/-2047, Outlander is
// offset by 10000, OpenInverter is percent times 10). It is not Nm.
// Using it in the formula above would slam the 12-bit field to full
// scale, so we report 0 Nm until a real Nm figure exists. The diesel
// used to send +219.5 Nm with the engine stopped. That was an artefact
// and we do not copy it.
// TODO: supply demand in Nm. FinalTorqueNm then applies the DSC limit.
float BMW_F11::DriverDemandNm() const { return 0.0f; }

// rpm to put on the bus. 0 until the inverter is actually in RUN
// after a start request. While it is running we never report below
// 750, so the dash and the DSC see an idling engine instead of a
// stalled one. 8000 is the diesel ECU's maximum: 8000 * 4 = 32000,
// which still fits the 16-bit rpm slot.
uint16_t BMW_F11::RpmTx() const {
  if (!EngineRunning())
    return 0;
  int rpm = motorRpm;
  if (rpm < 0)
    rpm = -rpm;
  if (rpm < 750)
    rpm = 750;
  if (rpm > 8000)
    rpm = 8000;
  return (uint16_t)rpm;
}

// Runs every 10 ms. Sends the fast engine frames, but only while
// PT-CAN is already awake. 20 ms frames go on even ticks. The diesel
// ECU used "phase tick 2"; at 20 ms that is the same slot. 0x08F is
// every 30 ms, sent when tick % 3 == 2, which is that phase.
void BMW_F11::Task10Ms() {
  AgeDscLimit();
  NoteBusState();
  ApplyBrake();
  UpdateBrakeRegen();

  // Start is remembered from here until the ignition drops, so the
  // running report can begin later, when precharge finishes and the
  // VCU enters RUN. It is not the running report itself.
  if (Param::GetBool(Param::din_brake) &&
      (kl50On || Param::GetBool(Param::din_start)))
    startSeen = true;
  if (!kl15On)
    startSeen = false;

  // Clear the MIL edge when we are not running, so the next RUN sends
  // the off frame again. "Running" is RUN mode after a start request,
  // not merely ignition on.
  bool running = EngineRunning();
  if (!running)
    milOffSent = false;

  if (!BusAwake())
    return;

  // DDE phase is tick 2 on a 10 ms grid. A 20 ms frame cannot tell
  // phase 2 from phase 0 at this tick, so the group goes on even ticks.
  // 0x8F is 30 ms, so phase 2 is tick % 3 == 2.
  if ((tick10 % 2) == 0)
    SendFastFrames();
  if ((tick10 % 3) == 2) {
    // 0x08F  PT-CAN  every 30 ms  wheel torque and "engine running".
    // E2E, data id 0x0020. DSC used to read the FlexRay twin of this.
    //   byte 0      CRC
    //   byte 1      alive in the low nibble, high nibble stays 1
    //   byte 2-3    wheel torque, little-endian, raw = Nm + 32000.
    //               0 Nm is 32000 (bytes 00 7D). Left at 0 Nm: we do
    //               not know the overall gear ratio. The diesel's
    //               -7 Nm idle value is not copied.
    //   byte 4      low nibble 0xC (12) = stopped, 2 = running.
    //               High nibble 7 is fill from the captures.
    //   byte 5      0x00
    //   byte 6      0x18, copied from the captures
    //   byte 7      0x10. High nibble 1 means ignition on / running.
    uint8_t b[8];
    b[1] = 0x10;
    b[2] = 0x00;
    b[3] = 0x7D;
    b[4] = running ? 0x72 : 0x7C; // low nibble 2 running, 12 stopped
    b[5] = 0x00;
    b[6] = 0x18;
    b[7] = 0x10; // high nibble 1: ignition on / running
    E2ESeal(b, 8, 0x0020, &alive8F);
    can->Send(0x08F, b, 8);
  }

  Send3F9(running);

  // MIL off in the same moment we first say the engine is running.
  // Never sent before the start request, and the ON frame is never sent.
  if (running && !milOffSent) {
    SendMilOff();
    milOffSent = true;
  }

  tick10++;
}

// Runs every 100 ms. Counts silence (20 ticks = 2 seconds) and, while
// the bus is awake, sends the slow frames: fuel used and 12 V.
void BMW_F11::Task100Ms() {
  if (busQuiet100 < 20)
    busQuiet100++;

  NoteBusState();
  if (!BusAwake())
    return;

  Send100MsFrames();
}

// The four frames the diesel ECU sent every 20 ms.
void BMW_F11::SendFastFrames() {
  uint16_t rpm = RpmTx();
  uint16_t trq = TorqueNmToRaw(FinalTorqueNm(DriverDemandNm()));
  GearCode gear = GearFromDir(Param::GetInt(Param::dir));

  // 0x0A5  PT-CAN  every 20 ms  engine rpm and torque.
  // This is what the dash rev counter follows. E2E, data id 0x0009.
  //   byte 0      CRC
  //   byte 1      alive in the low nibble, high nibble 0xF (unused bits)
  //   byte 2-4    torque twice, 12 bits each, same value.
  //               raw = (Nm + 1023.5) * 2. See TorqueNmToRaw.
  //   byte 5-6    rpm * 4, little-endian (0.25 rpm per bit)
  //   byte 7      low nibble 1 = speed valid, high nibble 0xF
  uint8_t a5[8];
  uint16_t rpm4 = (uint16_t)(rpm * 4); // 0xA5 bit 40, 0.25 rpm per bit
  a5[1] = 0xF0;                        // high nibble is fill
  a5[2] = (uint8_t)(trq & 0xFF);       // bit 16, torque 1
  a5[3] = (uint8_t)(((trq >> 8) & 0x0F) | ((trq & 0x0F) << 4));
  a5[4] = (uint8_t)((trq >> 4) & 0xFF); // bit 28, torque 2 = torque 1
  a5[5] = (uint8_t)(rpm4 & 0xFF);
  a5[6] = (uint8_t)(rpm4 >> 8);
  a5[7] = 0xF1; // status nibble at bit 56 = 1
  E2ESeal(a5, 8, 0x0009, &aliveA5);
  can->Send(0x0A5, a5, 8);

  // 0x0D9  PT-CAN  every 20 ms  accelerator pedal.
  // E2E, data id 0x0043. The high nibble of byte 1 must stay 1.
  // A 3 there meant "engine cranking", which an EV never sends.
  //   byte 0      CRC
  //   byte 1      alive in the low nibble, high nibble 1
  //   byte 2-3    pedal 1, 12 bits, low 8 in byte 2. Scale in PedalRaw.
  //               The high nibble of byte 3 stays 1 (capture constant).
  //   byte 4-5    pedal 2, same 12-bit value. High nibble of byte 5
  //               stays 0xE.
  //   byte 6-7    7F C0, copied from the captures
  uint16_t ped = PedalRaw();
  uint8_t d9[8];
  d9[1] = 0x10; // high nibble 1. 3 would mean cranking.
  d9[2] = (uint8_t)(ped & 0xFF);
  d9[3] = (uint8_t)(0x10 | ((ped >> 8) & 0x0F));
  d9[4] = (uint8_t)(ped & 0xFF);
  d9[5] = (uint8_t)(0xE0 | ((ped >> 8) & 0x0F));
  d9[6] = 0x7F;
  d9[7] = 0xC0;
  E2ESeal(d9, 8, 0x0043, &aliveD9);
  can->Send(0x0D9, d9, 8);

  // 0x0DC  PT-CAN  every 20 ms  selected gear. Six bytes.
  // E2E, data id 0x005A. Park / Reverse / Drive codes are in GearFromDir.
  //   byte 0      CRC
  //   byte 1      alive in the low nibble, high nibble 1
  //   byte 2      gear: 0x22 Park, 0x25 Reverse, 0x24 Drive
  //   byte 3-5    D0 07 FE. A speed-like field that was 0 in every
  //               capture, so it is copied unchanged.
  uint8_t dc[8] = {0};
  dc[1] = 0x10;
  dc[2] = gear.dc;
  dc[3] = 0xD0;
  dc[4] = 0x07;
  dc[5] = 0xFE;
  E2ESeal(dc, 6, 0x005A, &aliveDc);
  can->Send(0x0DC, dc, 6);

  // 0x0F3  PT-CAN  every 20 ms  a second, coarser rpm.
  // E2E, data id 0x00A6. Some ECUs cross-check this against 0x0A5.
  //   byte 0      CRC
  //   byte 1      low nibble = alive, high nibble = low 4 bits of rpm/10
  //   byte 2      the rest of that 12-bit rpm/10 value
  //   byte 3-7    C0 F0 FF FF FF, copied from the captures
  // 4094 is the top of the 12-bit field. 4095 would mean "no value".
  uint16_t n10 = (uint16_t)(rpm / 10); // rpm/10, 12 bits at bit 12
  if (n10 > 4094)
    n10 = 4094;
  uint8_t f3[8];
  f3[1] = (uint8_t)((n10 & 0x0F) << 4);
  f3[2] = (uint8_t)(n10 >> 4);
  f3[3] = 0xC0;
  f3[4] = 0xF0;
  f3[5] = 0xFF;
  f3[6] = 0xFF;
  f3[7] = 0xFF;
  E2ESeal(f3, 8, 0x00A6, &aliveF3);
  can->Send(0x0F3, f3, 8);
}

// 0x3F9  PT-CAN  every 1 s, and sooner if the contents change.
// Temperatures and "engine running" for the dash and the heater.
// E2E, data id 0x0037. Same contents as the FlexRay frame the Pico
// builds, but with its own counter. Never faster than 40 ms: the
// real ECU's shortest gap was about 33 ms.
//   byte 0      CRC
//   byte 1      alive in the low nibble, high nibble 1
//   byte 2      0x80 stopped, 0x82 running. 0x82 is sent only once the
//               VCU is in RUN after a start request. 0x81 meant
//               "cranking" and is never sent. Ignition on alone stays
//               0x80, so the CAS does not drop the starter early.
//   byte 3      0x64 stopped, 0x26 running. Bits 4-5 of this byte
//               (bits 28-29 of the frame) stay at 2, which means
//               "Park secured". The CAS watches that.
//   byte 4      coolant, degrees C + 48. While driving, clamped to
//               70..105 C so the gauge does not sit on cold or hot.
//   byte 5      oil, same +48 scale, from the motor temperature.
//               Not clamped to 70..105.
//   byte 6      low nibble = gear (3 Park, 2 Reverse, 5 Drive).
//               Bits 4-5 (frame bits 52-53, "start-ready") are 2
//               while stopped and 0 once we report running. The CAS
//               releases the starter (KL50) after it sees 0 here
//               together with byte 2 = 0x82.
//   byte 7      0x47, copied from the captures
void BMW_F11::Send3F9(bool ready) {
  GearCode gear = GearFromDir(Param::GetInt(Param::dir));

  float coolant = coolantC;
  if (ready) {
    if (coolant < 70.0f)
      coolant = 70.0f;
    if (coolant > 105.0f)
      coolant = 105.0f;
  }

  uint8_t b[8];
  b[1] = 0x10;
  b[2] = ready ? 0x82 : 0x80; // running / stopped. Never 0x81.
  b[3] = ready ? 0x26 : 0x64;
  // Bits 28-29 sit in bits 4-5 of this byte. Keep them at 2 ("Park
  // secured"). Both captured bytes already have that, this forces it.
  b[3] = (uint8_t)((b[3] & 0xCF) | 0x20);
  b[4] = TempPlus48(coolant);
  b[5] = TempPlus48(Param::GetFloat(Param::tmpm)); // oil, degrees C + 48
  b[6] = (uint8_t)(gear.nibble | (ready ? 0x00 : 0x20));
  b[7] = 0x47;

  bool changed = !seen3f9;
  for (int i = 0; i < 6; i++) {
    if (b[i + 2] != last3f9[i])
      changed = true;
  }
  if (seen3f9 && age3f910 < 255)
    age3f910++;

  // age3f910 counts 10 ms ticks. 100 = 1 second, 4 = 40 ms.
  bool due = !seen3f9 || age3f910 >= 100;
  bool gapOk = !seen3f9 || age3f910 >= 4;
  if (!(gapOk && (due || changed)))
    return;

  E2ESeal(b, 8, 0x0037, &alive3F9);
  can->Send(0x3F9, b, 8);
  for (int i = 0; i < 6; i++)
    last3f9[i] = b[i + 2];
  seen3f9 = true;
  age3f910 = 0;
}

// 0x592  PT-CAN  once, when the inverter enters Run.
// Check-control message 34, which is the MIL (check-engine light).
// The diesel ECU turned it on at ignition and off once the engine ran.
// We only ever send the off frame, so the lamp does not stay lit.
//   byte 0      0x40  check-control message, no extra data
//   byte 1-2    22 00  message id 34, little-endian
//   byte 3      0x14 = lamp off. 0x15 would be lamp on, and is never sent.
//   byte 4-7    FF FF FF FF
// Immobiliser frames on this id (they start with 0x80 or 0x85) are
// never sent, nor are 0x580, 0x5C0, 0x380 or 0x351.
void BMW_F11::SendMilOff() {
  uint8_t b[8] = {0x40, 0x22, 0x00, 0x14, 0xFF, 0xFF, 0xFF, 0xFF};
  can->Send(0x592, b, 8);
}

// The two frames the diesel ECU sent every 100 ms. Neither has E2E.
void BMW_F11::Send100MsFrames() {
  // 0x2C4  PT-CAN  every 100 ms  fuel used, for the trip computer.
  // Bytes 0-1 are a counter the dash differentiates to get consumption.
  // Held at 0, so the trip reads no fuel used. The tail is the constant
  // from the captures (three temperature-like bytes sitting at 0x64,
  // which is 0 C on a +100 scale, plus FF 00 F0).
  //   byte 0-1    fuel counter, little-endian, held at 0
  //   byte 2      0xFF
  //   byte 3-5    0x64 0x64 0x64
  //   byte 6-7    0x00 0xF0
  uint8_t fuel[8] = {0x00, 0x00, 0xFF, 0x64, 0x64, 0x64, 0x00, 0xF0};
  can->Send(0x2C4, fuel, 8);

  // 0x281  PT-CAN  every 100 ms  12 V supply, two bytes.
  // Power management reads this. 12-bit value in bits 0-11,
  // raw = volts / 0.015. The high nibble of byte 1 is 0xF.
  // 0x356 is the capture baseline, about 12.8 V (bytes 56 F3).
  // A measured 12.80 V encodes as 0x355, because 12.8 / 0.015
  // rounds to 853. Below 0.5 V we treat the pin as unwired and
  // send the baseline instead.
  //   byte 0      low 8 bits of the voltage
  //   byte 1      low nibble = high 4 bits, high nibble = 0xF
  float volts = Param::GetFloat(Param::uaux);
  uint16_t raw = 0x356;
  if (volts > 0.5f) {
    float scaled = volts / 0.015f;
    if (scaled > 4095.0f)
      scaled = 4095.0f;
    raw = (uint16_t)(scaled + 0.5f);
  }
  uint8_t v[8] = {0};
  v[0] = (uint8_t)(raw & 0xFF);
  v[1] = (uint8_t)(((raw >> 8) & 0x0F) | 0xF0);
  can->Send(0x281, v, 2);
}
