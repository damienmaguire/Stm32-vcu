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

#include "vehicles/BMW_E92.h"
#include "digio.h"
#include "stm32_can.h"

// Logged ambient-pressure byte. 0xC6 decodes as 994 hPa.
static const uint8_t kAmbientByte = 0xC6;

BMW_E92::BMW_E92()
    : motorRpm(0), coolantC(0.0f), sendOn(false), sawCar(false), saw130(false),
      kl15(false), kl50(false), awakeHold(0), lost130(0), tick(0), aliveA8(0),
      aliveA9(0), aliveAA(0), alive1D0(0), alive2F3(0), alive308(0), fuel(0),
      engineEverRun(false), busLive(false), nmPhase(E92NmWake),
      nmStatusFrames(0), nmWait(0), motorWasRun(false), runMs(0) {}

void BMW_E92::SetCanInterface(CanHardware *c) {
  can = c;
  // 0x130  terminal status from CAS, every 100 ms.
  // KL15 (ignition) and KL50 (crank) live in byte 0.
  can->RegisterUserMessage(0x130);
  // 0x1A0  vehicle speed from DSC, every 20 ms, 0.1 km/h per bit.
  can->RegisterUserMessage(0x1A0);
  // 0x480-0x4FF  network management from the other ECUs. One mask
  // filter. Used only as "the bus is awake", same job 0x1F3 does on
  // the E39. Our own 0x492 sits in this range and is ignored.
  can->RegisterUserMessage(0x480, 0x780);
}

bool BMW_E92::Ready() { return DigIo::t15_digi.Get() || kl15; }

bool BMW_E92::Start() { return Param::GetBool(Param::din_start) || kl50; }

bool BMW_E92::MotorRunning() {
  return Ready() && Param::GetInt(Param::opmode) == MOD_RUN;
}

E92Engine BMW_E92::Picture() {
  // Motor rpm comes from SetRevCounter (Param::speed). Throttle is
  // potnom, the same pedal the rest of the VCU already uses. Regen
  // (a negative potnom) is not an accelerator pedal.
  return E92EnginePicture(motorRpm, Param::GetFloat(Param::potnom), Ready());
}

void BMW_E92::Task10Ms() {
  if (!sendOn) {
    // Sleep: the next wake starts 0x492 at 12 01, and the "engine has
    // run" latches (0x0A8 byte 7, 0x1D0 byte 6) go back to the
    // before-start values.
    busLive = false;
    motorWasRun = false;
    engineEverRun = false;
    runMs = 0;
    return;
  }
  if (!busLive) {
    busLive = true;
    nmPhase = E92NmWake;
    nmStatusFrames = 0;
    nmWait = 0;
  }
  if (MotorRunning()) {
    if (!motorWasRun)
      runMs = 0;
    motorWasRun = true;
    engineEverRun = true;
    if (runMs < 60000u)
      runMs += 10;
  } else {
    motorWasRun = false;
    runMs = 0;
  }

  // 10 ms engine trio, every pass.
  Send0A8();
  Send0A9();
  Send0AA();

  // Slower frames counted off the same 10 ms tick, so the rates stay
  // on the log medians: 60, 100, 200, 1000 and 1800 ms.
  // 0x492 has its own counter so the wake frame goes out immediately.
  if ((tick % 6u) == 0u)
    Send1B6();
  if ((tick % 10u) == 0u)
    Send1D0();
  if ((tick % 20u) == 0u) {
    Send200();
    Send3B3();
  }
  if (nmWait == 0)
    Send492();
  nmWait++;
  if (nmWait >= 46)
    nmWait = 0;
  if ((tick % 50u) == 0u)
    Send308();
  if ((tick % 100u) == 0u)
    Send2F3();
  if ((tick % 180u) == 0u)
    Send3B4();
  tick++;
}

void BMW_E92::Task100Ms() {
  // Keep talking for a second after the last frame from the car, so a
  // single late NM frame (they come every 460 ms) does not look like
  // the bus went to sleep. Terminal 15, from the pin or from 0x130,
  // also keeps us talking, which is how the E39 decides.
  if (sawCar) {
    awakeHold = 10;
    sawCar = false;
  } else if (awakeHold > 0) {
    awakeHold--;
  }

  // 0x130 is every 100 ms. Five misses and we drop KL15 / KL50, so a
  // dead bus cannot leave the VCU thinking the key is still on.
  if (saw130) {
    lost130 = 0;
    saw130 = false;
  } else if (lost130 < 5) {
    lost130++;
    if (lost130 >= 5) {
      kl15 = false;
      kl50 = false;
    }
  }

  sendOn = DigIo::t15_digi.Get() || kl15 || (awakeHold > 0);
}

void BMW_E92::Send0A8() {
  E92Engine pic = Picture();
  E92PhaseBytes phase = E92Phase(MotorRunning(), engineEverRun, runMs);
  uint8_t bytes[8];
  // Both torque words here are the actual torque, same as 0x0AA and
  // word 4 of 0x0A9.
  E92Build0A8(bytes, aliveA8, pic.torque, phase.a8b5, phase.a8b7);
  aliveA8 = E92NextAlive(aliveA8);
  can->Send(0x0A8, bytes, 8);
}

void BMW_E92::Send0A9() {
  E92Engine pic = Picture();
  uint8_t bytes[8];
  E92Build0A9(bytes, aliveA9, pic.loss, pic.avail, pic.tmin, pic.torque);
  aliveA9 = E92NextAlive(aliveA9);
  can->Send(0x0A9, bytes, 8);
}

void BMW_E92::Send0AA() {
  E92Engine pic = Picture();
  E92PhaseBytes phase = E92Phase(MotorRunning(), engineEverRun, runMs);
  uint8_t bytes[8];
  E92Build0AA(bytes, aliveAA, pic, phase.aab7);
  aliveAA = E92NextAlive(aliveAA);
  can->Send(0x0AA, bytes, 8);
}

void BMW_E92::Send1B6() {
  // 0x1B6 every 60 ms, DLC 7. Running and off bytes are the logged
  // constants. "Running" is KL15 (or the T15 pin) and the VCU in RUN.
  uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  E92Build1B6(bytes, MotorRunning());
  can->Send(0x1B6, bytes, 7);
}

void BMW_E92::Send1D0() {
  // Coolant (byte 0) is the inverter heatsink temperature, the same
  // value the E39 class maps onto its gauge. Byte 1 is a different
  // sensor (intake air or ambient, same -48 scale). The logs show
  // 0x08 before the motor is running and about 12 C after that.
  uint8_t bytes[8];
  uint8_t air = MotorRunning() ? 0x3C : 0x08;
  E92PhaseBytes phase = E92Phase(MotorRunning(), engineEverRun, runMs);
  E92Build1D0(bytes, alive1D0, coolantC, air, kAmbientByte, fuel, phase.d0State,
              phase.d0b6, phase.d0b7);
  alive1D0 = E92NextAlive(alive1D0);
  if (Ready())
    fuel++;
  can->Send(0x1D0, bytes, 8);
}

void BMW_E92::Send200() {
  // 0x200 every 200 ms, DLC 8. Constant on the logged car.
  uint8_t bytes[8];
  E92Build200(bytes);
  can->Send(0x200, bytes, 8);
}

void BMW_E92::Send2F3() {
  uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  E92Build2F3(bytes, alive2F3);
  alive2F3 = E92NextAlive(alive2F3);
  can->Send(0x2F3, bytes, 3);
}

void BMW_E92::Send3B3() {
  // 0x3B3 every 200 ms (log median 201 ms), DLC 6.
  // Byte 1 ramps to 0xC8 over about 3 s once the motor is running.
  uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  E92Build3B3(bytes, MotorRunning(), E92PowerByte(runMs));
  can->Send(0x3B3, bytes, 6);
}

void BMW_E92::Send3B4() {
  // 0x3B4 every 1800 ms, DLC 8. On this car the DME is the sender
  // (the variant flag that would make it a receiver is clear).
  // The 12 V reading is Param::uaux, volts = raw * 0.015.
  uint8_t bytes[8];
  E92Build3B4(bytes, Param::GetFloat(Param::uaux));
  can->Send(0x3B4, bytes, 8);
}

void BMW_E92::Send308() {
  // 0x308 every 500 ms, DLC 3, only with the key on or the VCU in RUN.
  bool inRun = Param::GetInt(Param::opmode) == MOD_RUN;
  if (!Ready() && !inRun)
    return;
  uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  E92Build308(bytes, alive308);
  alive308 = E92NextAlive(alive308);
  can->Send(0x308, bytes, 3);
}

void BMW_E92::Send492() {
  // 0x492 every 460 ms, DLC 8. Logged sequence, bytes 2-7 always FF:
  //   12 01 once at wake (addressed to itself, opcode alive)
  //   17 02 once (opcode ring, next node 0x17)
  //   17 42 for about 10 s
  //   17 52 from then on (KL15 on, engine off or running)
  // 12 42 does not appear in the logs. We stop sending when the bus
  // goes quiet and terminal 15 is off, so this cannot hold the car awake.
  uint8_t bytes[8];
  E92Build492(bytes, nmPhase);
  nmPhase = E92NmNext(nmPhase, &nmStatusFrames);
  can->Send(0x492, bytes, 8);
}

void BMW_E92::DecodeCAN(int id, uint32_t *data) {
  uint8_t *bytes = (uint8_t *)data;

  if (id == 0x130) {
    // Byte 0 is four 2-bit fields, not a bit mask.
    //   bits 1:0  KL15, ignition. 1 = on.
    //   bits 3:2  run, set from crank until the stop request.
    //   bits 5:4  KL50, crank. 1 only while the starter is requested.
    //   bits 7:6  KL-R, the first wake. 1 once the car has woken.
    // A logged ignition-on frame is 0x41. A crank frame is 0x55.
    uint8_t b0 = bytes[0];
    kl15 = (b0 & 0x03) == 0x01;
    kl50 = ((b0 >> 4) & 0x03) == 0x01;
    saw130 = true;
    sawCar = true;
  } else if (id == 0x1A0) {
    // 12-bit speed in byte 0 and the low nibble of byte 1.
    // 0.1 km/h per bit. 0xFFF means "no speed".
    uint16_t raw = (uint16_t)(bytes[0] | ((bytes[1] & 0x0F) << 8));
    if (raw != 0x0FFF)
      Param::SetFloat(Param::Veh_Speed, raw * 0.1f);
    sawCar = true;
  } else if ((id & 0x780) == 0x480 && id != 0x492) {
    sawCar = true;
  }
}
