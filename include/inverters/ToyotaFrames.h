/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2019-2022 Damien Maguire <info@evbmw.com>
 * Changes by Tom de Bree <tom@voltinflux.com> 2026
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

#ifndef TOYOTAFRAMES_H
#define TOYOTAFRAMES_H

/* Frame building and decoding for the Toyota/Lexus hybrid inverter sync
 * serial link (HTM = VCU to inverter, MTH = inverter to VCU). No hardware
 * access, so it can be unit tested on the host. */

#include <stdint.h>

namespace ToyotaFrames {
constexpr int GS450H_HTM_LEN = 80;
constexpr int GS450H_MTH_LEN = 100;
constexpr int PRIUS_HTM_LEN = 100;
constexpr int PRIUS_MTH_LEN = 120;
constexpr int GS300H_HTM_LEN = 105;
constexpr int GS300H_MTH_LEN = 140;

struct MthData {
  int16_t dcBusVoltage;
  float tempInvWater;
  float tempInvInductor;
  int16_t mg1Speed;
  int16_t mg2Speed;
};

// 16-bit sum of all bytes before the last two, little-endian in the last two
inline void SetChecksum16(uint8_t *frame, int len) {
  uint16_t checksum = 0;

  for (int i = 0; i < (len - 2); i++)
    checksum += frame[i];
  frame[len - 2] = checksum & 0xFF;
  frame[len - 1] = checksum >> 8;
}

inline bool Checksum16Ok(const uint8_t *frame, int len) {
  uint16_t checksum = 0;

  for (int i = 0; i < (len - 2); i++)
    checksum += frame[i];
  return checksum == (frame[len - 2] | (frame[len - 1] << 8));
}

inline MthData DecodeMthGs450h(const uint8_t *mth) {
  MthData d;
  d.dcBusVoltage = ((mth[84] | mth[85] << 8) / 2);
  d.tempInvWater = int8_t(mth[42]);
  d.tempInvInductor = int8_t(mth[86]);
  d.mg1Speed = mth[6] | mth[7] << 8;
  d.mg2Speed = mth[31] | mth[32] << 8;
  return d;
}

inline MthData DecodeMthPrius(const uint8_t *mth) {
  MthData d;
  d.dcBusVoltage = (((mth[100] | mth[101] << 8) - 5) / 2);
  d.tempInvWater = int8_t(mth[20]); // from 300h
  d.tempInvInductor = (mth[86] | mth[87] << 8);
  d.mg1Speed = mth[6] | mth[7] << 8;
  d.mg2Speed = mth[38] | mth[39] << 8;
  return d;
}

inline MthData DecodeMthGs300h(const uint8_t *mth) {
  MthData d;
  d.dcBusVoltage = (((mth[117] | mth[118] << 8)) / 2);
  d.tempInvWater = int8_t(mth[20]);
  d.tempInvInductor = (mth[25] | mth[26] << 8);
  d.mg1Speed = mth[10] | mth[11] << 8;
  d.mg2Speed = mth[43] | mth[44] << 8;
  return d;
}

// Torque scale in percent: 0 while cut, otherwise ramps up to 100 by rampStep
inline float RampTorqueScale(float scale, bool cut, float rampStep) {
  if (cut)
    return 0;
  if (rampStep <= 0 || scale + rampStep >= 100)
    return 100;
  return scale + rampStep;
}

/* Tracks the MTH link. Bad checksums and timeouts both count as failures;
 * more than maxFailures in a row means the link is lost. While lost the
 * torque scale is 0, and once frames are good again it ramps back to 100%.
 * Starts out lost, so torque only ramps in after the first good frame. */
class CommMonitor {
public:
  explicit CommMonitor(uint16_t maxFailures)
      : maxFailures(maxFailures), failures(maxFailures + 1), torqueScale(0) {}

  void FrameOk() { failures = 0; }

  void FrameBad() {
    if (failures < UINT16_MAX)
      failures++;
    if (Lost())
      torqueScale = 0;
  }

  bool Lost() const { return failures > maxFailures; }
  uint16_t Failures() const { return failures; }

  // Call once per torque update, returns the torque scale in percent
  float UpdateTorqueScale(float rampStep) {
    torqueScale = RampTorqueScale(torqueScale, Lost(), rampStep);
    return torqueScale;
  }

private:
  uint16_t maxFailures;
  uint16_t failures;
  float torqueScale;
};

constexpr int OVERSPEED_HYSTERESIS = 500; // rpm

/* Cuts MG2 torque while MG2 turns faster than the limit, in either
 * direction. The cut holds until MG2 is OVERSPEED_HYSTERESIS below the
 * limit, then torque ramps back in. */
class OverspeedCut {
public:
  OverspeedCut() : cut(false), torqueScale(100) {}

  // Call once per torque update, returns the torque scale in percent
  float Update(int16_t mg2Speed, int limit, float rampStep) {
    int speed = mg2Speed;
    if (speed < 0)
      speed = -speed;

    if (speed > limit)
      cut = true;
    else if (speed < limit - OVERSPEED_HYSTERESIS)
      cut = false;

    torqueScale = RampTorqueScale(torqueScale, cut, rampStep);
    return torqueScale;
  }

  bool Cut() const { return cut; }

private:
  bool cut;
  float torqueScale;
};

// Check and decode a received MTH frame. Returns false and leaves out
// unchanged when the checksum is wrong.
inline bool ReadMthGs450h(const uint8_t *mth, MthData &out) {
  if (!Checksum16Ok(mth, GS450H_MTH_LEN))
    return false;
  out = DecodeMthGs450h(mth);
  return true;
}

inline bool ReadMthPrius(const uint8_t *mth, MthData &out) {
  if (!Checksum16Ok(mth, PRIUS_MTH_LEN))
    return false;
  out = DecodeMthPrius(mth);
  return true;
}

inline bool ReadMthGs300h(const uint8_t *mth, MthData &out) {
  if (!Checksum16Ok(mth, GS300H_MTH_LEN))
    return false;
  out = DecodeMthGs300h(mth);
  return true;
}

inline void BuildHtmGs450h(uint8_t *htm, int16_t mg1Torque, int16_t mg2Torque,
                           int16_t mg1Speed, int16_t mg2Speed) {
  // -3500 (reverse) to 3500 (forward)

  // speed feedback
  int16_t speedSum = mg2Speed + mg1Speed;
  speedSum /= 113;

  htm[0] = (uint8_t)speedSum;
  htm[75] = (mg1Torque * 4) & 0xFF;
  htm[76] = ((mg1Torque * 4) >> 8) & 0xFF;

  // mg1
  htm[5] = (-mg1Torque) & 0xFF; // negative is forward
  htm[6] = ((-mg1Torque) >> 8);
  htm[11] = htm[5];
  htm[12] = htm[6];

  // mg2
  htm[26] = (mg2Torque) & 0xFF; // positive is forward
  htm[27] = ((mg2Torque) >> 8) & 0xFF;
  htm[32] = htm[26];
  htm[33] = htm[27];

  htm[63] = (-5000) & 0xFF; // regen ability of battery
  htm[64] = ((-5000) >> 8);

  htm[65] = (27500) & 0xFF; // discharge ability of battery
  htm[66] = ((27500) >> 8);

  SetChecksum16(htm, GS450H_HTM_LEN);
}

/* Prius HTM bytes under test for the power limit. Compared with OEM frames,
 * 76-77 is the regen limit, 78-79 the discharge limit, 86 the ready flags and
 * 88 a "speed limit" like value. */
struct PriusLimits {
  int16_t regenLimit;     // bytes 76-77
  int16_t dischargeLimit; // bytes 78-79
  uint8_t byte86;
  uint8_t byte88;
};

// The values sent so far: 76-79 from the "forced zero" block, 86 and 88 = 137
constexpr PriusLimits PRIUS_DEFAULT_LIMITS = {9568, 1106, 137, 137};

// frameCount is the caller's frame counter, byte 94 steps every second frame
inline void BuildHtmPrius(uint8_t *htm, int16_t mg1Torque, int16_t mg2Torque,
                          int scaledTorqueTarget, uint8_t &frameCount,
                          const PriusLimits &limits) {
  // Bytes 75/76 seem to be MG1 for startup, but the relation to the MG1 bytes
  // is unclear (byte order flipped?), so they are not set from mg1Torque here.

  // mg1
  htm[5] = (mg1Torque) & 0xFF; // negative is forward
  htm[6] = ((mg1Torque) >> 8);
  htm[11] = htm[5];
  htm[12] = htm[6];

  // mg2 the MG2 values are now beside each other!
  htm[30] = (mg2Torque) & 0xFF; // positive is forward
  htm[31] = ((mg2Torque) >> 8) & 0xFF;

  if (scaledTorqueTarget > 0) {
    // forward direction these bytes should match
    htm[26] = htm[30];
    htm[27] = htm[31];
    htm[28] = (mg2Torque / 2) & 0xFF; // positive is forward
    htm[29] = ((mg2Torque / 2) >> 8) & 0xFF;
  }

  if (scaledTorqueTarget < 0) {
    // reverse direction these bytes should match
    htm[28] = htm[30];
    htm[29] = htm[31];
    htm[26] = (mg2Torque / 2) & 0xFF; // positive is forward
    htm[27] = ((mg2Torque / 2) >> 8) & 0xFF;
  }

  // Battery Limits = forced zero
  // 40	4	75	12	60	251	52	4
  htm[72] = 0x40;
  htm[73] = 0x04;
  htm[74] = 0x75;
  htm[75] = 0x12;
  htm[76] = limits.regenLimit & 0xFF;
  htm[77] = (limits.regenLimit >> 8) & 0xFF;
  htm[78] = limits.dischargeLimit & 0xFF;
  htm[79] = (limits.dischargeLimit >> 8) & 0xFF;

  htm[86] = limits.byte86; // 137 from start up
  htm[88] = limits.byte88; // 137, 221 on start

  if (++frameCount & 0x01) {
    htm[94]++;
  }

  SetChecksum16(htm, PRIUS_HTM_LEN);
}

inline void BuildHtmGs300h(uint8_t *htm, int16_t mg1Torque, int16_t mg2Torque) {
  // mg1
  htm[5] = (mg1Torque * -1) & 0xFF; // negative is forward
  htm[6] = ((mg1Torque * -1) >> 8);
  htm[11] = htm[5];
  htm[12] = htm[6];

  // mg2
  htm[31] = (mg2Torque) & 0xFF; // positive is forward
  htm[32] = ((mg2Torque) >> 8);
  htm[37] = htm[26];
  htm[38] = htm[27];

  // Battery Limits
  htm[79] = (-5000) & 0xFF; // regen ability of battery
  htm[80] = ((-5000) >> 8);

  htm[81] = (10000) & 0xFF; // discharge ability of battery
  htm[82] = ((10000) >> 8);

  SetChecksum16(htm, GS300H_HTM_LEN);
}
} // namespace ToyotaFrames

#endif /* TOYOTAFRAMES_H */
