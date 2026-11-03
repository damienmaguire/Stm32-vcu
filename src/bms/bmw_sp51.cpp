/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2026 Damien Maguire <info@evbmw.com>
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

/* BMW SP51 (Gen5 PHEV) battery, Preh EA443 SME.
 *
 * Frames and timing come from Ian's bench tool sp51ctl.py and the
 * close_v4 / close_v7 playbacks, which closed a real pack. Signals marked
 * FALLBACK below have a guessed scaling; treat them with care.
 *
 * The pack must never hear silence once it is awake: about 1.2-1.8 s
 * without our frames latches its CAN off until the 12 V is cycled. So
 * everything here is sent every tick from power up, whatever the state.
 */

#include "bms/bmw_sp51.h"
#include "digio.h"
#include "errormessage.h"

// 0x53A close sequence states (bytes 5 and 6 change).
#define STAGE_A 0 // idle
#define STAGE_B 1 // about to close
#define STAGE_C 2 // close
#define STAGE_D 3 // closed

#define TICKS_LOST 50     // 0x112 missing this long = pack gone (0.5 s)
#define TICKS_GAP 10      // a gap longer than this restarts the settle
#define TICKS_REQUEST 200 // 0x53A state B for 2 s before the close
#define TICKS_CLOSE 500   // give the pack 5 s to report closed
#define TICKS_LEFT 100    // allow 1 s out of "closed" before aborting
#define TICKS_WAKE 300    // no answer after 3 s: go quiet once and retry
#define TICKS_QUIET 100   // 1 s of silence for that retry

// CRC-8 SAE J1850 (poly 0x1D, no final XOR) over bytes 1..len-1, start
// value depends on the frame id.
static uint8_t Crc8(const uint8_t *data, uint8_t len, uint8_t init) {
  uint8_t crc = init;
  for (uint8_t i = 1; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
  }
  return crc;
}

// Alive counters run 0..14 and wrap.
static uint8_t NextAlive(uint8_t alive) { return alive >= 14 ? 0 : alive + 1; }

static uint16_t U16(const uint8_t *data, int i) {
  return data[i] | (data[i + 1] << 8);
}

void BMW_SP51::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(0x112); // pack voltage, current, link voltage 20ms
  can->RegisterUserMessage(0x1FA); // contactor state, cell temps 1s + events
  can->RegisterUserMessage(0x2F5); // voltage and current limits
  can->RegisterUserMessage(0x40D); // power limits
  can->RegisterUserMessage(0x432); // displayed SOC
}

void BMW_SP51::DecodeCAN(int id, uint8_t *data) {
  uint8_t *bytes = (uint8_t *)data;

  switch (id) {
  case 0x112:
    // 0xFFF0 and up is "not valid" (0xFFFD shows up once at most closes,
    // which would read as 5734 A): keep the last good value.
    if (U16(bytes, 0) < 0xFFF0)
      packAmps = U16(bytes, 0) * 0.1f - 819.2f;
    if (U16(bytes, 2) != 0xFFFF)
      packVolts = U16(bytes, 2) * 0.1f;
    if (bytes[7] != 0xFF)
      linkVolts = bytes[7] * 4.0f; // ANALYSIS, about 4 V per bit
    // Open requests, 2 bits each, 1 = no request (always 1 in our logs,
    // b6 reads 0x75). Anything else, 0 included, is treated as "open".
    openReq = bytes[5] >> 6;           // b5 bits 6-7, normal open
    openInstant = bytes[6] & 0x03;     // b6 bits 0-1, FALLBACK
    openFast = (bytes[6] >> 2) & 0x03; // b6 bits 2-3, FALLBACK
    age112 = 0;
    seen112 = true;
    break;
  case 0x1FA:
    last1FA = bytes[1]; // 0x45 = open and ready, high nibble 6 = closed
    if (bytes[1] == 0x45) {
      if (n45 < 255)
        n45 = n45 + 1;
    } else {
      n45 = 0;
    }
    tempMin = bytes[6] - 50; // ANALYSIS
    tempMax = bytes[7] - 50; // ANALYSIS
    break;
  case 0x2F5:
    uChargeMax = U16(bytes, 0) * 0.1f;          // ANALYSIS
    iChargeMax = U16(bytes, 2) * 0.1f - 819.2f; // FALLBACK
    uDischargeMin = U16(bytes, 4) * 0.1f;       // ANALYSIS
    iDischargeMax = U16(bytes, 6) * 0.1f;       // ANALYSIS
    break;
  case 0x40D: // FALLBACK, which word is short and long term is a guess
  {
    float chgShort = U16(bytes, 0) * 3 / 1000.0f;
    float dchgShort = U16(bytes, 2) * 3 / 1000.0f;
    float chgLong = U16(bytes, 4) * 3 / 1000.0f;
    float dchgLong = U16(bytes, 6) * 3 / 1000.0f;
    pChargeKw = chgShort < chgLong ? chgShort : chgLong;
    pDischargeKw = dchgShort < dchgLong ? dchgShort : dchgLong;
    break;
  }
  case 0x432:
    soc = bytes[4]; // FALLBACK, 0xFF = not valid
    break;
  default:
    return;
  }
  seenPack = true;
}

// The pack's contactors follow the ZV's own contactor outputs, so they
// open exactly when external contactors would, delays included:
//   Off, from Run/Charge  stay closed for the ZV's 2.5 s shutdown delay
//                         (rlyDly) so the inverter, DC-DC, heater etc.
//                         are off before the contactors open, then open
//   Precharge             run the close sequence. The ZV sees the precharge
//                         as done when udc reaches udcsw, which is only
//                         allowed once the pack reports closed (Task100Ms).
//   Run, Charge,          stay closed (prec_out / dcsw_out are on)
//   Preheat, Maintain
//   PchFail               open (the ZV clears prec_out)
//   Over voltage trip     open as soon as the ZV drops its outputs
// Open means 0x10B OPEN and 0x53A A; sending carries on regardless.
// The precharge opmode is checked too, because the ZV turns prec_out on
// only 250 ms after entering it and the close needs every moment of the
// ZV precharge timeout.
// Pack-side faults (0x112 lost, pack asks to open) still open at once,
// and the pack then stays open until the ZV outputs are off. They post
// ERR_BMSCOMM and the charge and discharge limits go to 0 at once.
// ERR_BMSCOMM only shows in lasterr and the error log: nothing in the ZV
// acts on it, so the opmode stays Run (or Charge) with the pack open until
// the key goes off or charging ends.
bool BMW_SP51::CloseRequested() {
  return Param::GetInt(Param::opmode) == MOD_PRECHARGE ||
         DigIo::prec_out.Get() || DigIo::dcsw_out.Get();
}

bool BMW_SP51::PackValid() { return seen112 && age112 <= TICKS_LOST; }

// The pack asks to open with any of its three open request fields.
bool BMW_SP51::PackOpenRequest() {
  return openReq != 1 || openInstant != 1 || openFast != 1;
}

// Close only when 0x112 has run SP51Settle seconds (default 20, as every
// proven bench close had) without a gap, and the last three 0x1FA frames
// said "open and ready" (0x45).
bool BMW_SP51::PackSettled() {
  return steady112 >= Param::GetInt(Param::SP51Settle) * 100 && n45 >= 3;
}

void BMW_SP51::Abort(Fault why) {
  closeCmd = 0;
  stage53A = STAGE_A;
  state = FAULT;
  fault = why;
  ErrorMessage::Post(ERR_BMSCOMM); // lasterr only, the opmode is unchanged
  // Limits to 0 now, not at the next Task100Ms.
  Param::SetFloat(Param::BMS_ChargeLim, 0);
  Param::SetFloat(Param::BMS_MaxInput, 0);
  Param::SetFloat(Param::BMS_MaxOutput, 0);
  Param::SetFloat(Param::SP51IDchg, 0);
}

// Fresh start after the wake retry: counters and clock from the top.
void BMW_SP51::Restart() {
  tick = 0;
  alive10B = alive12F = alive1A1 = 0;
  seconds328 = 0;
  days328 = 0;
  closeCmd = 0;
  stage53A = STAGE_A;
}

void BMW_SP51::Task10Ms() {
  if (!can)
    return;
  if (age112 < 0xFFFF)
    age112 = age112 + 1;

  // Wake retry: only while the pack has never answered. If the pack
  // answers during the silence, talk again at once: a silence of 1.2 s or
  // more after it has answered latches it.
  if (silentTicks > 0) {
    if (seenPack) {
      silentTicks = 0;
      Restart();
    } else {
      silentTicks--;
      if (silentTicks == 0)
        Restart();
      return;
    }
  }
  if (!seenPack && !wakeRetried && tick >= TICKS_WAKE) {
    wakeRetried = true;
    silentTicks = TICKS_QUIET;
    return;
  }

  if (age112 <= TICKS_GAP) {
    if (steady112 < 0xFFFF)
      steady112++;
  } else {
    steady112 = 0;
  }

  RunStateMachine();
  SendFrames();
  tick++;
}

void BMW_SP51::RunStateMachine() {
  bool request = CloseRequested();
  bool valid = PackValid();
  bool slot = (tick % 20) == 0; // the next 0x53A goes out this tick

  // Checked every tick while a close is under way.
  if (state == REQUEST || state == CLOSING || state == CLOSED) {
    if (!valid) {
      Abort(PACK_LOST);
    } else if (PackOpenRequest()) {
      Abort(OPEN_REQ);
    } else if (!request) {
      closeCmd = 0; // normal open
      stage53A = STAGE_A;
      state = STANDBY;
    }
  }

  stepTicks++;

  switch (state) {
  case WAKE:
  case STANDBY:
  case READY:
  case LOST:
    if (!seenPack)
      state = WAKE;
    else if (!valid) {
      if (state != LOST)
        ErrorMessage::Post(ERR_BMSCOMM); // lasterr only
      state = LOST;
    } else if (!PackSettled())
      state = STANDBY;
    else if (request && slot) {
      state = REQUEST;
      stage53A = STAGE_B;
      fault = NONE;
      stepTicks = 0;
    } else
      state = READY;
    break;

  case REQUEST:
    if (stepTicks >= TICKS_REQUEST && slot) {
      // Last look before the close, as sp51ctl.py does.
      if (last1FA == 0x45 && !PackOpenRequest() && packVolts > 200) {
        state = CLOSING;
        closeCmd = 1;
        stage53A = STAGE_C; // SendFrames moves on to D after two frames
        framesC = 0;
        stepTicks = 0;
      } else {
        Abort(PRECHECK_FAIL);
      }
    }
    break;

  case CLOSING:
    if ((last1FA >> 4) == 6) {
      state = CLOSED;
      notClosedTicks = 0;
    } else if (stepTicks > TICKS_CLOSE) {
      Abort(CLOSE_TIMEOUT);
    }
    break;

  case CLOSED:
    if ((last1FA >> 4) == 6)
      notClosedTicks = 0;
    else if (++notClosedTicks > TICKS_LEFT)
      Abort(LEFT_CLOSED);
    break;

  case FAULT:
    // Stay open until the ZV contactor outputs are off, then start again.
    if (!request)
      state = STANDBY;
    break;
  }
}

void BMW_SP51::SendFrames() {
  uint8_t bytes[8];

  // Car frames the pack needs. Never on the car's PT-CAN: with the F11
  // vehicle on the same bus the car (or the F11 class) already sends them.
  bool support =
      Param::GetBool(Param::SP51Supp) &&
      !(Param::GetInt(Param::BMSCan) == Param::GetInt(Param::VehicleCan) &&
        Param::GetInt(Param::Vehicle) == vBMW_F11);

  // 0x12F terminal status, a static KL15 on, every 100 ms. The pack does
  // not decode it (it may only be the wake), so it does not follow the key.
  if (support && tick % 10 == 0) {
    bytes[1] = 0x20 | alive12F;
    bytes[2] = 0x8A;
    bytes[3] = 0xDD;
    bytes[4] = 0xF1;
    bytes[5] = 0x05;
    bytes[6] = 0x30;
    bytes[7] = 0x01;
    bytes[0] = Crc8(bytes, 8, 0x60);
    can->Send(0x12F, bytes, 8);
    alive12F = NextAlive(alive12F);
  }

  // 0x10B contactor request (high nibble of byte 1), every 20 ms.
  if (tick % 2 == 0) {
    bytes[1] = (closeCmd << 4) | alive10B;
    bytes[2] = 0xFC;
    bytes[0] = Crc8(bytes, 3, 0x3F);
    can->Send(0x10B, bytes, 3);
    alive10B = NextAlive(alive10B);
  }

  // 0x53A close sequence, every 200 ms.
  if (tick % 20 == 0) {
    bytes[0] = 0x40;
    bytes[1] = 0x3A;
    bytes[2] = 0x04;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = stage53A == STAGE_A ? 0x00 : (stage53A == STAGE_B ? 0x80 : 0x84);
    bytes[6] = stage53A == STAGE_D ? 0x00 : 0x01;
    bytes[7] = 0x00;
    can->Send(0x53A, bytes, 8);
    // C goes out twice (400 ms), then D, as in every playback that closed
    // the pack.
    if (stage53A == STAGE_C && ++framesC >= 2)
      stage53A = STAGE_D;
  }

  // Once a second, spread over the odd ticks to keep the send buffer short.
  if (support) {
    switch (tick % 100) {
    case 1: // 0x328 vehicle clock: seconds and days, both little endian
    {
      uint32_t sec = 243785948 + seconds328;
      bytes[0] = sec;
      bytes[1] = sec >> 8;
      bytes[2] = sec >> 16;
      bytes[3] = sec >> 24;
      bytes[4] = (9244 + days328);
      bytes[5] = (9244 + days328) >> 8;
      can->Send(0x328, bytes, 6);
      seconds328++;
      if ((243785948 + seconds328) % 86400 == 0)
        days328++;
      break;
    }
    case 3: {
      uint8_t f[8] = {0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
      can->Send(0x3A0, f, 8);
      break;
    }
    case 5: {
      uint8_t f[8] = {0xB7, 0x60, 0x01, 0x0F, 0x0F, 0x31, 0xFF, 0xFF};
      can->Send(0x3CA, f, 8);
      break;
    }
    case 7: {
      uint8_t f[8] = {0x6E, 0x6F};
      can->Send(0x2CA, f, 2);
      break;
    }
    case 9: {
      uint8_t f[8] = {0xF1, 0xFF};
      can->Send(0x3E8, f, 2);
      break;
    }
    case 11: {
      uint8_t f[8] = {0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x10};
      can->Send(0x510, f, 8);
      break;
    }
    case 13: {
      uint8_t f[8] = {0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x12};
      can->Send(0x512, f, 8);
      break;
    }
    case 15: {
      uint8_t f[8] = {0xA0, 0x86, 0x01, 0x06, 0x00, 0x0D, 0xBD, 0x07};
      can->Send(0x330, f, 8);
      break;
    }
    }
  }

  // 0x1A1 vehicle speed, a static standstill (b4 0x81), every 20 ms. Real
  // speed would also need the moving qualifier (0xA1): untested, so not
  // sent.
  if (support && tick % 2 == 0) {
    bytes[1] = 0xC0 | alive1A1;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x81;
    bytes[0] = Crc8(bytes, 5, 0x77);
    can->Send(0x1A1, bytes, 5);
    alive1A1 = NextAlive(alive1A1);
  }
}

// Pack reports its contactors closed (0x1FA state 6) with fresh data.
bool BMW_SP51::PackClosed() { return PackValid() && (last1FA >> 4) == 6; }

// Limits are only given while we hold the pack closed (state CLOSED) and
// the pack agrees (0x1FA state 6). After an abort our state changes at
// once, while 0x1FA takes about 80 ms to report open.
bool BMW_SP51::LimitsAllowed() { return state == CLOSED && PackClosed(); }

// Charge current limit for the chargers. The current scaling is a
// FALLBACK, so also cap it by the power limit. 0 unless the pack is closed.
float BMW_SP51::MaxChargeCurrent() {
  if (!LimitsAllowed() || packVolts < 200)
    return 0;
  float limit = iChargeMax;
  float fromPower = pChargeKw * 1000 / packVolts;
  if (fromPower < limit)
    limit = fromPower;
  return limit > 0 ? limit : 0;
}

void BMW_SP51::Task100Ms() {
  bool valid = PackValid();
  bool closed = PackClosed();
  bool limits = LimitsAllowed();
  int shunt = Param::GetInt(Param::ShuntType);

  // 0x112 has two voltages. U_Pack (b2-3) is the cell stack side, before
  // the contactors: it reads the full pack voltage even when open. b7 is
  // the link (output) side after the contactors, 0 when open, about 4 V
  // per bit (ANALYSIS). As in the Leaf class, udc2 is the battery side.
  // udc is the bus side, the one the ZV compares with udcsw to finish
  // precharge.
  if (shunt == 0) // Only populate if no shunt is used
  {
    Param::SetFloat(Param::udc, valid ? linkVolts : 0);
    if (valid) {
      Param::SetFloat(Param::udc2, packVolts);
      Param::SetFloat(Param::idc, packAmps);
      Param::SetFloat(Param::power, packVolts * packAmps / 1000);
      if (soc <= 100)
        Param::SetFloat(Param::SOC, soc);
    }
  }
  if (shunt != 4) // ISA_udcsw sets udcsw itself
  {
    // Precharge is done when the pack reports closed and the link is within
    // 30 V of the pack. Until then (or without pack data) udcsw is held at
    // 500 V, as STW does, so the ZV keeps waiting or times out.
    if (closed && packVolts > 200)
      Param::SetFloat(Param::udcsw, packVolts - 30);
    else
      Param::SetFloat(Param::udcsw, 500);
  }

  Param::SetFloat(Param::BMS_ChargeLim, MaxChargeCurrent());
  // Cell voltages (BMS_Vmin, BMS_Vmax) are only on UDS: left unset.
  Param::SetFloat(Param::BMS_Tmin, valid ? tempMin : 0);
  Param::SetFloat(Param::BMS_Tmax, valid ? tempMax : 0);
  // No average on CAN: use the middle of the coldest and warmest cell.
  Param::SetFloat(Param::BMS_Tavg, valid ? (tempMin + tempMax) / 2 : 0);
  // FALLBACK: lower of the two 0x40D words for each direction (kW).
  // 0 unless the pack is closed, so nothing draws or charges otherwise.
  Param::SetFloat(Param::BMS_MaxInput, limits ? pChargeKw : 0);
  Param::SetFloat(Param::BMS_MaxOutput, limits ? pDischargeKw : 0);

  Param::SetInt(Param::SP51State, state);
  Param::SetInt(Param::SP51Fault, fault);
  Param::SetInt(Param::SP51HvSt, valid ? last1FA >> 4 : 0);
  Param::SetFloat(Param::SP51ULink, valid ? linkVolts : 0);
  Param::SetFloat(Param::SP51UMax, valid ? uChargeMax : 0);
  Param::SetFloat(Param::SP51UMin, valid ? uDischargeMin : 0);
  Param::SetFloat(Param::SP51IDchg, limits ? iDischargeMax : 0);
}
