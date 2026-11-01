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

// Start asleep: nothing is sent until the car speaks first.
BMW_F11::BMW_F11()
    : motorRpm(0), coolantC(0), nowMs(0), lastRxMs(0), wakeMs(0), wakeSeq(0),
      wakeSeqSeen(0), rxEver(false), busWasAwake(false), foreignTx(false),
      kl15Ever(false), onMs(0), offMs(0), next592(0), final592(false),
      startFault(false), stKl(0), stKlMs(0), stKlHave(false), brakeDownMs(0),
      brakeDownHave(false), eng(), tick200(0), tick10(0), fastTick(0),
      fastOn(false), tick8F(0), on8F(false), last8F7(0x80), aliveA5(0),
      aliveD9(0), aliveDc(0), aliveF3(0), alive8F(0), alive3F9(0),
      seen3f9(false), next3f9(0), last3f9Ms(0), kl15On(false),
      kl50On(false), brakePressed(false), brakeOneSwitch(false),
      brakePressure(false), brakeSeen(false), brakeAge10ms(0), brakeTorqueNm(0),
      brakeTorqueSeen(false), brakeTorqueAge10ms(0), dscLimitFresh(false), dscLimitAge10ms(0), dscLimitNm(0),
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
  // 0x3F9 and 0x0A5 are frames WE send. If they arrive, another box
  // (an old Pico build, a real DDE) is sending them too, and we stop.
  // v1h.3 ptcan_emu_core.c pte_on_rx, PTE_FAULT_FOREIGN.
  can->RegisterUserMessage(0x3F9);
  can->RegisterUserMessage(0x0A5);

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

// PT-CAN just woke (W). Restart the alive counters so the next frame
// carries 1, and every frame's timeline from W (v1h.3 new_wake).
void BMW_F11::OnBusWake() {
  busWasAwake = true;
  tick10 = 0;
  tick200 = 0;
  fastOn = false;
  on8F = false;
  last8F7 = 0x80;
  eng = EngStory();
  startFault = false;
  aliveA5 = 0;
  aliveD9 = 0;
  aliveDc = 0;
  aliveF3 = 0;
  alive8F = 0;
  alive3F9 = 0;
  seen3f9 = false;
}

// One second with no 0x12F, 0x173 or 0x0EF (we stopped talking after
// 200 ms already). The next frame is a new wake. Forget
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
}

// Called every 1 ms. Runs OnBusWake once per new wake (DecodeCAN saw
// the first frame after >= 1 s of silence) and OnBusSleep once after
// 1 s of silence.
void BMW_F11::NoteBusState() {
  if (wakeSeq != wakeSeqSeen) {
    wakeSeqSeen = wakeSeq;
    OnBusWake();
    return;
  }
  if (busWasAwake && (nowMs - lastRxMs) >= 1000)
    OnBusSleep();
}

// v1h.3 bus gate (ptcan_emu_core.c pte_state): we may talk while one
// of the car's frames came in the last 200 ms, and nobody else sends
// our ids.
bool BMW_F11::BusAwake() const {
  return rxEver && !foreignTx && (nowMs - lastRxMs) <= 200;
}

// One frame from the car. >= 1 s after the last one it is a new wake
// (W): the ignition history is forgotten here, in the receive path, so
// that this very frame (often a 0x12F) is decoded into the new wake.
void BMW_F11::NoteRx() {
  uint32_t now = nowMs;
  if (!rxEver || (now - lastRxMs) >= 1000) {
    wakeMs = now;
    kl15On = false;
    kl50On = false;
    kl15Ever = false;
    stKlHave = false;
    final592 = false;
    wakeSeq++;
  }
  rxEver = true;
  lastRxMs = now;
}

// A frame arrived. Only 0x12F and 0x173 prove that PT-CAN is up.
// Other ids (inverter, charger) are ignored for that test.
void BMW_F11::DecodeCAN(int id, uint32_t *data) {
  uint8_t *bytes = (uint8_t *)data;

  // These three ids are PT-CAN from the car. Inverter and charger
  // frames must not count, or we could be the box that wakes the bus.
  if (id == 0x12F || id == 0x173 || id == 0x0EF)
    NoteRx();
  // Someone else sends one of our frames: stay silent (F11Fault 1).
  if (id == 0x3F9 || id == 0x0A5)
    foreignTx = true;

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
  bool wasOn = kl15On;

  if (st >= 0x0A || (st == 0x09 && kl15On))
    kl15On = true;
  else
    kl15On = false;

  if (st >= 0x0D || (st == 0x0C && kl50On))
    kl50On = true;
  else
    kl50On = false;

  // For the engine story (v1h.3 eng_input_stkl): the raw ST_KL, when
  // the CAS ended its crank request, and the byte 6 brake mirror.
  uint32_t now = nowMs;
  if (eng.startD && !eng.dEnded && st != 0x0D &&
      (eng.phase == 1 || eng.phase == 2))
    eng.dEnded = true;
  stKl = st;
  stKlMs = now;
  stKlHave = true;
  if ((bytes[6] & 0x03) == 0x03) {
    brakeDownMs = now;
    brakeDownHave = true;
  }
  // KL15 edges start the frame timelines (v1h.3 pte_on_rx).
  if (kl15On && !wasOn) {
    kl15Ever = true;
    onMs = now;
    next592 = now + 98; // first 0x592 at ON+98 ms
    final592 = false;
  } else if (!kl15On && wasOn) {
    offMs = now;
    next592 = now + 54; // one last 0x592 at OFF+54 ms
    final592 = true;
    startFault = false;
  }
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
    brakeDownMs = nowMs; // for the engine story and the gear logic
    brakeDownHave = true;
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

// The bus picture of a running engine: our engine story is in RUN
// (0x3F9 byte 2 = 0x82). It starts only from a start request with the
// brake down, so KL15 alone stays "engine stopped".
bool BMW_F11::EngineRunning() const { return eng.phase == 2; }

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

// Motor rpm for the bus. 0 unless the engine story is running. The
// story's own idle (780 rpm) wins while the motor turns slower, see
// EngineOutputs. 8000 is the diesel ECU's maximum: 8000 * 4 = 32000,
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
// PT-CAN is awake. The 20 ms group starts at W+462 ms and 0x08F (30 ms)
// at W+459 ms, the v1h.3 first-frame times, then keeps its own grid.
void BMW_F11::Task10Ms() {
  AgeDscLimit();
  ApplyBrake();
  UpdateBrakeRegen();

  if (!BusAwake())
    return;

  uint32_t w = SinceWake();
  if (!fastOn && w >= 462) {
    fastOn = true;
    fastTick = tick10;
  }
  if (fastOn && ((uint16_t)(tick10 - fastTick) % 2) == 0)
    SendFastFrames();
  if (!on8F && w >= 459) {
    on8F = true;
    tick8F = tick10;
  }
  if (on8F && ((uint16_t)(tick10 - tick8F) % 3) == 0)
    Send08F();

  tick10++;
}

// 0x08F  PT-CAN  every 30 ms  wheel torque and "engine running".
// E2E, data id 0x0020. v1h.3 template `CC 1x 00 7d 8c 00 18 b7`.
//   byte 1      alive in the low nibble, high nibble 1
//   byte 2-3    00 7D (0 Nm). F9 7C from 100 ms after the first rpm
//               of a start (sensors valid), kept after the stop.
//   byte 4      0x8C until a start, 0x72 from first rpm + 7 ms,
//               0x7C once the engine stops (dde_engine.c r29_b5).
//   byte 5-6    00 18
//   byte 7      0x80 before KL15, 0x10 from ON+15 ms, 0xC0 from OFF+31 ms
void BMW_F11::Send08F() {
  uint8_t b[8] = {0, 0x10, 0x00, 0x7D, 0x8C, 0x00, 0x18, 0x80};
  uint8_t b7 = 0x80;
  if (kl15On)
    b7 = SinceOn() >= 15 ? 0x10 : last8F7;
  else if (kl15Ever)
    b7 = SinceOff() >= 31 ? 0xC0 : 0x10;
  b[7] = b7;
  last8F7 = b7;
  if (eng.active) {
    if (eng.f97c) {
      b[2] = 0xF9;
      b[3] = 0x7C;
    }
    if (eng.r29b5)
      b[4] = eng.r29b5;
  }
  E2ESeal(b, 8, 0x0020, &alive8F);
  can->Send(0x08F, b, 8);
}

// Runs every 100 ms: fuel used and 12 V, from W+611 ms.
void BMW_F11::Task100Ms() {
  if (!BusAwake() || SinceWake() < 611)
    return;

  Send100MsFrames();
}

// The four frames the diesel ECU sent every 20 ms. Templates are the
// v1h.3 ones (ptcan_emu_core.c build()); the engine story patches them
// (dde_engine.c patch_can_engine).
void BMW_F11::SendFastFrames() {
  // 0x0A5  PT-CAN  every 20 ms  engine rpm and torque.
  // This is what the dash rev counter follows. E2E, data id 0x0009.
  //   byte 0      CRC
  //   byte 1      alive in the low nibble, high nibble 0xF (unused bits)
  //   byte 2-4    torque twice, 12 bits each, same value.
  //               raw = (Nm + 1023.5) * 2. See TorqueNmToRaw.
  //   byte 5-6    rpm * 4, little-endian (0.25 rpm per bit)
  //   byte 7      0xF1
  // Without a start: `b8 87 7b 00 00 f1` before KL15, `1a aa a1 ..`
  // from ON+55 ms, `19 9a a1 ..` after KL15 off (captured values).
  static const uint8_t a5Pre[6] = {0xB8, 0x87, 0x7B, 0x00, 0x00, 0xF1};
  static const uint8_t a5On[6] = {0x1A, 0xAA, 0xA1, 0x00, 0x00, 0xF1};
  static const uint8_t a5Off[6] = {0x19, 0x9A, 0xA1, 0x00, 0x00, 0xF1};
  const uint8_t *t = kl15On ? (SinceOn() >= 55 ? a5On : a5Pre)
                            : (kl15Ever ? a5Off : a5Pre);
  uint8_t a5[8];
  a5[1] = 0xF0; // high nibble is fill
  for (int i = 0; i < 6; i++)
    a5[i + 2] = t[i];
  if (eng.active) {
    uint16_t trq = eng.torque;
    // TODO: real Nm. DriverDemandNm() is 0 today, so the story's
    // torque (the captured idle values) is what goes out.
    if (EngineRunning() && DriverDemandNm() != 0.0f)
      trq = TorqueNmToRaw(FinalTorqueNm(DriverDemandNm()));
    if (trq) {
      a5[2] = (uint8_t)(trq & 0xFF); // bit 16, torque 1
      a5[3] = (uint8_t)(((trq >> 8) & 0x0F) | ((trq & 0x0F) << 4));
      a5[4] = (uint8_t)((trq >> 4) & 0xFF); // bit 28, torque 2 = torque 1
    }
    a5[5] = (uint8_t)(eng.rpm4 & 0xFF);
    a5[6] = (uint8_t)(eng.rpm4 >> 8);
  }
  E2ESeal(a5, 8, 0x0009, &aliveA5);
  can->Send(0x0A5, a5, 8);

  // 0x0D9  PT-CAN  every 20 ms  accelerator pedal.
  // E2E, data id 0x0043. v1h.3 sends a released pedal:
  // `CC 1x 00 10 00 e0 7f c0`. The high nibble of byte 1 is 3 during
  // the crank marker of a start (first 105 ms), else 1.
  //   byte 2-3    pedal 1, 12 bits, low 8 in byte 2. Scale in PedalRaw.
  //               The high nibble of byte 3 stays 1 (capture constant).
  //   byte 4-5    pedal 2, same 12-bit value. High nibble of byte 5
  //               stays 0xE.
  //   byte 6-7    7F C0, copied from the captures
  // F11Pedal = 1 puts the real pedal in. Default 0 = released, as v1h.3.
  uint16_t ped = Param::GetInt(Param::F11Pedal) ? PedalRaw() : 0;
  uint8_t d9[8];
  d9[1] = (eng.active && eng.cntHi) ? 0x30 : 0x10;
  d9[2] = (uint8_t)(ped & 0xFF);
  d9[3] = (uint8_t)(0x10 | ((ped >> 8) & 0x0F));
  d9[4] = (uint8_t)(ped & 0xFF);
  d9[5] = (uint8_t)(0xE0 | ((ped >> 8) & 0x0F));
  d9[6] = 0x7F;
  d9[7] = 0xC0;
  E2ESeal(d9, 8, 0x0043, &aliveD9);
  can->Send(0x0D9, d9, 8);

  // 0x0DC  PT-CAN  every 20 ms  selected gear. Six bytes.
  // E2E, data id 0x005A. Wake ramp of the real DDE (bytes 2-5):
  //   80 FF FF F0 until W+502 ms, 20 D0 07 F1 until W+703 ms,
  //   then 22 D0 07 FE (0x22 = Park).
  // Automatic: bytes 2 and 5 follow our gear (Step 2). Manual: the
  // template stays (22 .. FE is also the neutral code).
  uint32_t w = SinceWake();
  uint8_t dc[8] = {0};
  dc[1] = 0x10;
  if (w < 502) {
    dc[2] = 0x80; dc[3] = 0xFF; dc[4] = 0xFF; dc[5] = 0xF0;
  } else if (w < 703) {
    dc[2] = 0x20; dc[3] = 0xD0; dc[4] = 0x07; dc[5] = 0xF1;
  } else {
    dc[2] = 0x22; dc[3] = 0xD0; dc[4] = 0x07; dc[5] = 0xFE;
  }
  E2ESeal(dc, 6, 0x005A, &aliveDc);
  can->Send(0x0DC, dc, 6);

  // 0x0F3  PT-CAN  every 20 ms  a second, coarser rpm.
  // E2E, data id 0x00A6. Some ECUs cross-check this against 0x0A5.
  //   byte 1      low nibble = alive, high nibble = low 4 bits of rpm/10
  //   byte 2      the rest of that 12-bit rpm/10 value
  //   byte 3-7    C0 F0 FF FF FF, copied from the captures
  // rpm/10 follows the story's rpm with a 200 ms lag (dde_engine.c f3).
  uint16_t n10 = eng.active ? eng.f3val : 0;
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

// 0x3F9  PT-CAN  every 1 s from W+439 ms, plus a copy as soon as the
// state bytes change (1 ms task, v1h.3 gap 1 ms).
// Temperatures and "engine running" for the dash and the heater.
// E2E, data id 0x0037.
//   byte 1      alive in the low nibble, high nibble 1 (3 during the
//               crank marker, the first 105 ms of a start)
//   byte 2      0x80 stopped, 0x81 cranking, 0x82 running, 0x92 stopping
//   byte 3      0x64 stopped, 0x65 cranking, 0x26 running, 0x66 stopping.
//               Bits 4-5 (frame bits 28-29) = P lock: 2 = "Park
//               secured", which the CAS watches. Automatic: from our
//               gear (Step 2). Manual: 0 (no P on a manual box).
//   byte 4      coolant, degrees C + 48. While driving, clamped to
//               70..105 C so the gauge does not sit on cold or hot.
//   byte 5      oil, same +48 scale, from the motor temperature.
//   byte 6      low nibble = gear (3 P, 2 R, 1 N, 5 D). Manual: 1.
//               High nibble = start-ready: 2 before any start, 1 while
//               cranking and stopped, 0 while running. The CAS
//               releases the starter when it sees 0 with 0x82.
//   byte 7      0x47
// Wake ramp of the real DDE (v1h.3 build()): byte 6 00 -> 10 (W+459)
// -> 20 (W+509) -> 23 (W+709); byte 3 34 -> 74 (W+529) -> 64 (W+609).
// Only bytes 2, 3, 6 and 7 trigger a change copy. The temperatures go
// out with the next frame (a float wobbling across a 1 C step must
// not send a frame every 1 ms).
void BMW_F11::Send3F9(bool ready) {
  uint32_t w = SinceWake();
  if (w < 439)
    return;

  float coolant = coolantC;
  if (ready) {
    if (coolant < 70.0f)
      coolant = 70.0f;
    if (coolant > 105.0f)
      coolant = 105.0f;
  }

  uint8_t b[8];
  b[1] = 0x10;
  b[2] = 0x80;
  b[3] = w < 529 ? 0x34 : (w < 609 ? 0x74 : 0x64);
  b[4] = TempPlus48(coolant);
  b[5] = TempPlus48(Param::GetFloat(Param::tmpm)); // oil, degrees C + 48
  b[6] = w < 459 ? 0x00 : (w < 509 ? 0x10 : (w < 709 ? 0x20 : 0x23));
  b[7] = 0x47;
  if (eng.active) {
    if (eng.cntHi)
      b[1] = (uint8_t)(eng.cntHi << 4);
    if (eng.e6b3) {
      b[2] = eng.e6b3;
      b[3] = eng.e6b4;
    }
    if (eng.e6ready != 0xFF)
      b[6] = (uint8_t)((b[6] & 0x0F) | (eng.e6ready << 4));
  }
  if (!AutoBox()) {
    // Manual: no P lock and gear "N" once the wake ramp has set them.
    // Not captured on a manual car yet (see BMW_F11.h).
    if (w >= 609)
      b[3] = (uint8_t)(b[3] & 0xCF);
    if (w >= 709)
      b[6] = (uint8_t)((b[6] & 0xF0) | 0x01);
  }

  bool changed = seen3f9 && (b[2] != last3f9[0] || b[3] != last3f9[1] ||
                             b[6] != last3f9[4] || b[7] != last3f9[5]);
  bool due = !seen3f9 || (int32_t)(nowMs - next3f9) >= 0;
  if (!due && !changed)
    return;
  if (!seen3f9) {
    next3f9 = nowMs + 1000;
  } else if (due) {
    // keep the 1 s grid; a change copy does not move it
    next3f9 += 1000;
    if ((int32_t)(nowMs - next3f9) >= 0)
      next3f9 = nowMs + 1000;
  }

  E2ESeal(b, 8, 0x0037, &alive3F9);
  can->Send(0x3F9, b, 8);
  for (int i = 0; i < 6; i++)
    last3f9[i] = b[i + 2];
  seen3f9 = true;
  last3f9Ms = nowMs;
}

// 0x592  PT-CAN  check-control message 34, the MIL (check-engine
// light). v1h.3 timeline: first at ON+98 ms, then every 5.1 s while
// KL15 is on, and one last frame at OFF+54 ms.
//   byte 0      0x40  check-control message, no extra data
//   byte 1-2    22 00  message id 34, little-endian
//   byte 3      0x14 = lamp off. The real DDE sent 0x15 after its
//               immobiliser exchange; not proven to be "lamp on", so
//               never sent (as v1h.3).
//   byte 4-7    FF FF FF FF
// Immobiliser frames on this id (they start with 0x80 or 0x85) are
// never sent, nor are 0x580, 0x5C0, 0x380 or 0x351.
void BMW_F11::Send592() {
  if (!kl15On && !final592)
    return;
  if ((int32_t)(nowMs - next592) < 0)
    return;
  uint8_t b[8] = {0x40, 0x22, 0x00, 0x14, 0xFF, 0xFF, 0xFF, 0xFF};
  can->Send(0x592, b, 8);
  if (!kl15On)
    final592 = false;
  next592 = nowMs + 5100;
}

// The two frames the diesel ECU sent every 100 ms. Neither has E2E.
void BMW_F11::Send100MsFrames() {
  // 0x2C4  PT-CAN  every 100 ms  fuel used, for the trip computer.
  // Bytes 0-1 are a counter the dash differentiates to get consumption.
  // The tail is the constant
  // from the captures (three temperature-like bytes sitting at 0x64,
  // which is 0 C on a +100 scale, plus FF 00 F0).
  //   byte 0-1    fuel counter, little-endian
  //   byte 2      0xFF
  //   byte 3-5    0x64 0x64 0x64
  //   byte 6-7    0x00 0xF0
  // After a start the counter grows with the story's rpm, as v1h.3.
  uint8_t fuel[8] = {0x00, 0x00, 0xFF, 0x64, 0x64, 0x64, 0x00, 0xF0};
  if (eng.active) {
    fuel[0] = (uint8_t)(eng.fuel & 0xFF);
    fuel[1] = (uint8_t)(eng.fuel >> 8);
  }
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

// ---------------------------------------------------------------------
// Added for the v1h.3 timing (pico-flexray v1h.3, spec
// pico_zv_private_can.md section 5). Small pieces, in the order the
// 1 ms task runs them.
// ---------------------------------------------------------------------

// ms from t to now, never negative.
static uint32_t SinceMs(uint32_t now, uint32_t t) {
  int32_t d = (int32_t)(now - t);
  return d < 0 ? 0u : (uint32_t)d;
}

// Runs every 1 ms. The bus gate, the engine story (stepped every 5 ms,
// once per FlexRay cycle like the Pico), the start watchdog, and the
// frames whose timing has to be exact.
void BMW_F11::Task1Ms() {
  nowMs++;
  NoteBusState();
  if ((nowMs % 5) == 0)
    EngineStep();
  EngineWatchdog();

  // No torque unless our engine story says running. SelectDirection
  // only runs every 200 ms, so cut the direction here at once.
  if (!EngineRunning() && Param::GetInt(Param::dir) != 0)
    Param::SetInt(Param::dir, 0);

  Param::SetInt(Param::F11Eng, eng.phase);
  Param::SetInt(Param::F11Fault, foreignTx ? 1 : (startFault ? 2 : 0));

  if (!BusAwake())
    return;
  Send3F9(EngineRunning());
  Send592();
}

// Runs every 200 ms: the 1 s constant frames, from W+611 ms.
void BMW_F11::Task200Ms() {
  if (!BusAwake() || SinceWake() < 611)
    return;
  if ((tick200++ % 5) != 0)
    return;
  // 0x1B9  PT-CAN  every 1 s  DDE, constant `00 ff` (v1h.3).
  uint8_t b9[8] = {0x00, 0xFF};
  can->Send(0x1B9, b9, 2);
  // 0x396  PT-CAN  every 1 s  EGS, constant `fd ff`. Automatic only.
  if (AutoBox()) {
    uint8_t g[8] = {0xFD, 0xFF};
    can->Send(0x396, g, 2);
  }
}

// Gear for the VCU (utils::SelectDirection asks the vehicle first).
// While our engine story is not running we say NEUTRAL, which is
// direction 0 and therefore zero torque. (PARK would be direction 2,
// and the throttle code multiplies by the direction.)
// Manual box with the story running: return false, so the normal
// forward / reverse inputs decide.
bool BMW_F11::GetGear(Vehicle::gear &outGear) {
  if (!EngineRunning()) {
    outGear = Vehicle::NEUTRAL;
    return true;
  }
  return false;
}

// Brake down now (pin or CAN), or a pressed 0x173 / 0x12F mirror
// within the last ms milliseconds.
bool BMW_F11::BrakeRecent(uint32_t ms) const {
  if (Param::GetBool(Param::din_brake))
    return true;
  return brakeDownHave && SinceMs(nowMs, brakeDownMs) <= ms;
}

// One engine step (v1h.3 dde_engine.c eng_step).
void BMW_F11::EngineStep() {
  uint32_t dt = eng.stepHave ? SinceMs(nowMs, eng.lastStep) : 5u;
  if (dt > 1000u)
    dt = 1000u;
  eng.lastStep = nowMs;
  eng.stepHave = true;
  EngineCheckStop();
  EngineCheckStart();
  EngineOutputs(dt);
}

// Start (v1h.3 check_start, CAS mode): ST_KL D (crank), fresh within
// 300 ms, and the brake down within the last 2 s. The wired start
// button also starts it, with ST_KL A or D and the brake within 300 ms
// (v1h.3 fallback mode).
void BMW_F11::EngineCheckStart() {
  if (eng.phase != 0 && eng.phase != 4)
    return;
  if (!stKlHave || SinceMs(nowMs, stKlMs) > 300)
    return;
  if (stKl != 0x0D)
    eng.wdBlock = false; // the crank request ended: a new start is allowed
  if (eng.wdBlock)
    return;
  bool cas = stKl == 0x0D && BrakeRecent(2000);
  bool pin = Param::GetBool(Param::din_start) &&
             (stKl == 0x0A || stKl == 0x0D) && BrakeRecent(300);
  if (!cas && !pin)
    return;
  eng.phase = 1; // crank
  eng.t0 = nowMs;
  eng.curRun = false;
  eng.t9Have = false;
  eng.startD = stKl == 0x0D;
  eng.dEnded = false;
  eng.t82Have = false;
  eng.wdOk = false;
}

// Stop (v1h.3 check_stop): ST_KL 8 or lower; ST_KL 9 held for 1 s;
// no ST_KL for 600 ms; or the CAS ended D before we said 0x82 (it
// refused the start, so we must not claim the engine started).
void BMW_F11::EngineCheckStop() {
  if (eng.phase != 1 && eng.phase != 2)
    return;
  if (eng.startD && eng.dEnded && !eng.t82Have) {
    EngineStopNow();
    return;
  }
  if (!stKlHave || SinceMs(nowMs, stKlMs) > 600) {
    EngineStopNow();
    return;
  }
  if (stKl >= 0x0A) {
    eng.t9Have = false;
    return;
  }
  if (stKl == 0x09) {
    if (!eng.t9Have) {
      eng.t9Have = true;
      eng.t9 = nowMs;
    }
    if (SinceMs(nowMs, eng.t9) >= 1000)
      EngineStopNow();
    return;
  }
  EngineStopNow();
}

void BMW_F11::EngineStopNow() {
  if (!eng.curRun && eng.phase == 1) {
    // aborted before the first rpm: nothing was shown yet
    eng.phase = eng.hasRun ? 4 : 0;
    return;
  }
  eng.rpmAtStop = eng.rpm4;
  eng.ts = nowMs;
  eng.phase = 3; // stop (run-down)
}

// Start watchdog: we said "running" (0x82), but the VCU has not
// reached RUN 5 s later. Stop the story, so the car does not think an
// engine is turning that cannot drive, and show F11Fault = 2. No new
// start until ST_KL leaves D.
void BMW_F11::EngineWatchdog() {
  if (eng.phase != 2)
    return;
  if (Param::GetInt(Param::opmode) == MOD_RUN) {
    eng.wdOk = true;
    return;
  }
  if (!eng.wdOk && SinceMs(nowMs, eng.t82) >= 5000) {
    startFault = true;
    eng.wdBlock = true;
    EngineStopNow();
  }
}

// v1h.3 SHORT start profile (dde_engine.c eng_profiles[ENG_PROF_SHORT]),
// ms after the start trigger: crank marker 0..105, rpm from 0, first
// fire at 50, run-up 130 ms, "injecting" from 40, 0x92 2 ms into a stop.
static const uint32_t kBlipOff = 105, kFire = 50, kRunup = 130;
static const uint32_t kT92 = 2, kRpm82 = 950, kFlareEnd = 1500;
static const uint32_t kIdleRpm = 780;

// exp(-x/10) * 1000 for x = 0..50 (dde_engine.c exp_tab)
static const uint16_t kExpTab[51] = {
    1000, 905, 819, 741, 670, 607, 549, 497, 449, 407, 368, 333, 301,
    273,  247, 223, 202, 183, 165, 150, 135, 122, 111, 100, 91,  82,
    74,   67,  61,  55,  50,  45,  41,  37,  33,  30,  27,  25,  22,
    20,   18,  17,  15,  14,  12,  11,  10,  9,   8,   7,   7};

// 0..amp..0 triangle (dde_engine.c tri)
static uint32_t Tri(uint32_t t, uint32_t period, uint32_t amp) {
  uint32_t ph = t % period, half = period / 2u;
  return ph < half ? amp * ph / half : amp * (period - ph) / half;
}

// rpm * 4 while starting / running, t = ms since the start
// (dde_engine.c rpm4_start): crank 115..210 rpm, run-up 200 -> 1000,
// flare 1000-1060 until 1.5 s, then settle to 780 rpm idle.
static uint32_t Rpm4Start(uint32_t t) {
  if (t < kFire)
    return 4u * (115u + t * 60u / kFire + Tri(t, 140u, 35u));
  if (t < kFire + kRunup)
    return 4u * (200u + (t - kFire) * 800u / kRunup);
  if (t < kFlareEnd)
    return 4u * (1000u + Tri(t, 300u, 60u));
  uint32_t x = (t - kFlareEnd) / 100u;
  uint32_t over = x > 50u ? 0u : 260u * kExpTab[x] / 1000u;
  return 4u * (kIdleRpm - 1u + over) + Tri(t, 1200u, 8u);
}

// Run-down after a stop as per mille of the rpm at the stop, t = ms
// since the stop (dde_engine.c stop_tab, captured).
static uint32_t Rpm4Stop(uint32_t rpm4AtStop, uint32_t t) {
  static const uint16_t ms[9] = {0, 380, 402, 542, 661, 762, 882, 1180, 1200};
  static const uint16_t pm[9] = {1000, 1000, 912, 651, 441, 359, 92, 77, 0};
  for (int k = 1; k < 9; k++) {
    if (t < ms[k]) {
      int32_t p = pm[k - 1] + ((int32_t)pm[k] - (int32_t)pm[k - 1]) *
                                  (int32_t)(t - ms[k - 1]) /
                                  (int32_t)(ms[k] - ms[k - 1]);
      return rpm4AtStop * (uint32_t)p / 1000u;
    }
  }
  return 0;
}

// What the story puts on the bus right now (dde_engine.c set_outputs,
// the PT-CAN part only).
void BMW_F11::EngineOutputs(uint32_t dtMs) {
  EngStory &e = eng;
  uint32_t now = nowMs;
  e.active = false;
  e.cntHi = 0;
  e.e6b3 = 0;
  e.e6b4 = 0;
  e.e6ready = 0xFF;
  e.rpm4 = 0;
  e.torque = 0;
  e.r29b5 = 0;
  e.f97c = false;
  if (e.phase == 0 && !e.hasRun) {
    e.f3q8 = 0; // not started in this wake: templates only
    return;
  }
  e.active = true;
  switch (e.phase) {
  case 1: // crank
  case 2: // run
  {
    uint32_t t = SinceMs(now, e.t0);
    e.rpm4 = (uint16_t)Rpm4Start(t);
    if (e.rpm4 && !e.curRun) {
      e.curRun = true;
      e.hasRun = true;
      e.trpm = now;
    }
    if (t < kBlipOff)
      e.cntHi = 3; // crank marker
    if (e.phase == 1 && e.rpm4 >= 4u * kRpm82) {
      e.phase = 2; // 0x82 from here
      e.t82 = now;
      e.t82Have = true;
    }
    if (e.phase == 1) {
      e.e6b3 = 0x81;
      e.e6b4 = 0x65;
      e.e6ready = 1;
      e.torque = t < kFire ? 2560 : (uint16_t)(2560u - (t - kFire) * 420u / kRunup);
    } else {
      e.e6b3 = 0x82;
      e.e6b4 = 0x26;
      e.e6ready = 0;
      e.torque = t < kFlareEnd ? 2140 : 2098;
      // After the start flare the rev counter follows the motor when it
      // turns faster than the 780 rpm idle.
      uint32_t real = 4u * RpmTx();
      if (t >= kFlareEnd && real > e.rpm4)
        e.rpm4 = (uint16_t)real;
    }
    if (e.curRun) {
      uint32_t tr = SinceMs(now, e.trpm); // ms since the first rpm
      if (tr >= 7u)
        e.r29b5 = 0x72;
      if (tr >= 100u)
        e.f97c = true;
    } else if (e.hasRun) {
      // restart after an earlier run, before this start's first rpm:
      // the "stopped" values stay until the first rpm
      e.r29b5 = 0x7C;
      e.f97c = true;
      if (!e.e6b3) {
        e.e6b3 = 0x80;
        e.e6b4 = 0x64;
        e.e6ready = 1;
      }
      if (!e.torque)
        e.torque = 1978;
    }
    break;
  }
  case 3: // stop: run-down
  {
    uint32_t ts = SinceMs(now, e.ts);
    e.rpm4 = (uint16_t)Rpm4Stop(e.rpmAtStop, ts);
    e.e6b3 = ts >= kT92 ? 0x92 : 0x82;
    e.e6b4 = 0x66;
    e.e6ready = 0;
    e.torque = ts < 400u ? 2098 : 1947;
    e.r29b5 = ts >= kT92 ? 0x7C : 0x72;
    e.f97c = true;
    if (e.rpm4 != 0 || ts < 1200u)
      break;
    e.phase = 4;
    e.t80 = now;
  }
  /* fall through */
  case 4: // stopped
    e.rpm4 = 0;
    e.e6b3 = 0x80;
    e.e6b4 = 0x64;
    e.e6ready = SinceMs(now, e.t80) >= 20u ? 1 : 0;
    e.torque = 1978;
    e.r29b5 = 0x7C;
    e.f97c = true;
    break;
  default:
    break;
  }

  // 0x0F3: rpm/10 with about 0.2 s lag (dde_engine.c f3_q8)
  uint32_t target = (uint32_t)e.rpm4 * 256u / 40u;
  int32_t diff = (int32_t)target - (int32_t)e.f3q8;
  e.f3q8 = (uint32_t)((int32_t)e.f3q8 +
                      diff * (int32_t)(dtMs > 200u ? 200u : dtMs) / 200);
  if (e.rpm4 == 0 && e.f3q8 < 256u)
    e.f3q8 = 0;
  e.f3val = (uint16_t)(e.f3q8 / 256u);
  // 0x2C4 fuel counter (dde_engine.c fuel_q10)
  if (e.rpm4) {
    uint32_t rpm = e.rpm4 / 4u;
    uint32_t per100 = (e.phase == 1 && rpm < 400u) ? rpm / 10u : rpm / 20u;
    e.fuelQ10 += per100 * dtMs * 10u / 100u;
    e.fuel = (uint16_t)(e.fuelQ10 / 10u);
  }
}
