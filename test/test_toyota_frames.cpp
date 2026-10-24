/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2026 Tom de Bree <tom@voltinflux.com>
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

#include "inverters/ToyotaFrames.h"
#include "params.h"
#include "test_list.h"
#include "toyota_fixtures.h"
#include <string.h>

using namespace std;
using namespace ToyotaFrames;

struct Fixture {
  const uint8_t *frame;
  int len;
};

static const Fixture fixtures[] = {
    {GS450H_HTM_PEAK_ACCEL, GS450H_HTM_LEN}, {GS450H_HTM_SEED, GS450H_HTM_LEN},
    {PRIUS_HTM_BENCH_INIT, PRIUS_HTM_LEN},   {AURIS_MTH_1800, PRIUS_MTH_LEN},
    {AURIS_MTH_2700, PRIUS_MTH_LEN},         {PHV_HTM_INIT, GS300H_HTM_LEN},
    {PHV_HTM_READY, GS300H_HTM_LEN},         {PHV_HTM_DRIVE, GS300H_HTM_LEN},
    {PHV_MTH_DRIVE, GS300H_MTH_LEN},
};
static const int numFixtures = sizeof(fixtures) / sizeof(fixtures[0]);

// CHECKSUM
static void TestChecksum16AcceptsAllCapturedFrames() {
  bool allOk = true;
  for (int i = 0; i < numFixtures; i++)
    allOk = allOk && Checksum16Ok(fixtures[i].frame, fixtures[i].len);
  ASSERT(allOk);
}

static void TestChecksum16RejectsSingleBitErrors() {
  uint8_t buf[GS300H_MTH_LEN];
  bool allRejected = true;
  for (int i = 0; i < numFixtures; i++) {
    for (int pos = 0; pos < fixtures[i].len; pos++) {
      memcpy(buf, fixtures[i].frame, fixtures[i].len);
      buf[pos] ^= 1 << (pos % 8);
      allRejected = allRejected && !Checksum16Ok(buf, fixtures[i].len);
    }
  }
  ASSERT(allRejected);
}

static void TestChecksum16RejectsShiftedFrames() {
  uint8_t buf[GS300H_MTH_LEN + 1];
  bool allRejected = true;
  for (int i = 0; i < numFixtures; i++) {
    // frame received one byte late, with a leading byte from the line
    buf[0] = 0xFF;
    memcpy(buf + 1, fixtures[i].frame, fixtures[i].len);
    allRejected = allRejected && !Checksum16Ok(buf, fixtures[i].len);
  }
  ASSERT(allRejected);
}

static void TestChecksum16RejectsTruncatedFrames() {
  bool allRejected = true;
  for (int i = 0; i < numFixtures; i++)
    allRejected =
        allRejected && !Checksum16Ok(fixtures[i].frame, fixtures[i].len - 1);
  ASSERT(allRejected);
}

static void TestSetChecksum16MatchesCapturedFrames() {
  uint8_t buf[GS300H_MTH_LEN];
  bool allMatch = true;
  for (int i = 0; i < numFixtures; i++) {
    const int len = fixtures[i].len;
    memcpy(buf, fixtures[i].frame, len);
    buf[len - 2] = 0;
    buf[len - 1] = 0;
    SetChecksum16(buf, len);
    allMatch = allMatch && memcmp(buf, fixtures[i].frame, len) == 0;
  }
  ASSERT(allMatch);
}

// MTH DECODING
static void TestDecodeMthGs300hOnPriusPhvFrame() {
  MthData d = DecodeMthGs300h(PHV_MTH_DRIVE);
  ASSERT(d.dcBusVoltage == 229 && d.mg1Speed == 22 && d.mg2Speed == -211 &&
         d.tempInvWater == 18 && d.tempInvInductor == 17);
}

static void TestDecodeMthPriusOnAurisFrames() {
  MthData d1 = DecodeMthPrius(AURIS_MTH_1800);
  MthData d2 = DecodeMthPrius(AURIS_MTH_2700);
  ASSERT(d1.dcBusVoltage == 238 && d1.mg2Speed == 257 &&
         d1.tempInvWater == 15 && d2.mg2Speed == -219);
}

// MTH RECEIVE CHECK
static bool SameMth(const MthData &a, const MthData &b) {
  return a.dcBusVoltage == b.dcBusVoltage && a.mg1Speed == b.mg1Speed &&
         a.mg2Speed == b.mg2Speed && a.tempInvWater == b.tempInvWater &&
         a.tempInvInductor == b.tempInvInductor;
}

// No GS450h MTH capture yet, so build a valid frame by hand
static void MakeGs450hMth(uint8_t *mth) {
  memset(mth, 0, GS450H_MTH_LEN);
  mth[1] = 12;
  mth[6] = 0x2C; // MG1 300 rpm
  mth[7] = 0x01;
  mth[31] = 0xD0; // MG2 2000 rpm
  mth[32] = 0x07;
  mth[42] = 25;
  mth[84] = 0x20; // 800 / 2 = 400 V
  mth[85] = 0x03;
  SetChecksum16(mth, GS450H_MTH_LEN);
}

static void TestReadMthAcceptsValidFrames() {
  uint8_t gs450h[GS450H_MTH_LEN];
  MakeGs450hMth(gs450h);
  MthData a, b, c;
  ASSERT(ReadMthGs450h(gs450h, a) && a.mg1Speed == 300 && a.mg2Speed == 2000 &&
         a.dcBusVoltage == 400 && ReadMthPrius(AURIS_MTH_1800, b) &&
         SameMth(b, DecodeMthPrius(AURIS_MTH_1800)) &&
         ReadMthGs300h(PHV_MTH_DRIVE, c) &&
         SameMth(c, DecodeMthGs300h(PHV_MTH_DRIVE)));
}

// A good frame followed by a corrupt one: the corrupt one must be rejected
// and the values from the good one kept
static void TestReadMthRejectsCorruptFrameAfterGoodFrame() {
  uint8_t gs450h[GS450H_MTH_LEN], prius[PRIUS_MTH_LEN], gs300h[GS300H_MTH_LEN];
  MthData a, b, c;
  MakeGs450hMth(gs450h);
  memcpy(prius, AURIS_MTH_1800, PRIUS_MTH_LEN);
  memcpy(gs300h, PHV_MTH_DRIVE, GS300H_MTH_LEN);
  bool goodRead = ReadMthGs450h(gs450h, a) && ReadMthPrius(prius, b) &&
                  ReadMthGs300h(gs300h, c);
  const MthData lastA = a, lastB = b, lastC = c;

  gs450h[32] ^= 0x40; // MG2 speed high byte
  prius[39] ^= 0x40;
  gs300h[44] ^= 0x40;
  bool rejected = !ReadMthGs450h(gs450h, a) && !ReadMthPrius(prius, b) &&
                  !ReadMthGs300h(gs300h, c);
  ASSERT(goodRead && rejected && SameMth(a, lastA) && SameMth(b, lastB) &&
         SameMth(c, lastC));
}

// The old code also accepted an 8-bit sum in the last byte
static void TestReadMthRejectsFrameWithOnly8BitChecksum() {
  uint8_t mth[PRIUS_MTH_LEN];
  memcpy(mth, AURIS_MTH_2700, PRIUS_MTH_LEN);
  mth[38] ^= 0x01;
  uint8_t sum8 = 0;
  for (int i = 0; i < PRIUS_MTH_LEN - 1; i++)
    sum8 += mth[i];
  mth[PRIUS_MTH_LEN - 1] = sum8;
  MthData d;
  ASSERT(!ReadMthPrius(mth, d));
}

// COMM LOSS
static void TestCommMonitorStartsLostWithNoTorque() {
  CommMonitor comm(10);
  ASSERT(comm.Lost() && comm.UpdateTorqueScale(10) == 0);
}

static void TestCommMonitorRampsTorqueInAfterGoodFrame() {
  CommMonitor comm(10);
  comm.FrameOk();
  bool ramp = true;
  for (int i = 1; i <= 10; i++)
    ramp = ramp && comm.UpdateTorqueScale(10) == i * 10;
  ASSERT(!comm.Lost() && ramp && comm.UpdateTorqueScale(10) == 100);
}

static void TestCommMonitorLostAfterMoreThanMaxFailures() {
  CommMonitor comm(10);
  comm.FrameOk();
  comm.UpdateTorqueScale(100);
  for (int i = 0; i < 10; i++)
    comm.FrameBad();
  bool stillUp = !comm.Lost() && comm.UpdateTorqueScale(100) == 100;
  comm.FrameBad();
  ASSERT(stillUp && comm.Lost() && comm.UpdateTorqueScale(100) == 0);
}

static void TestCommMonitorGoodFrameResetsFailureCount() {
  CommMonitor comm(10);
  for (int i = 0; i < 10; i++)
    comm.FrameBad();
  comm.FrameOk();
  for (int i = 0; i < 10; i++)
    comm.FrameBad();
  ASSERT(!comm.Lost() && comm.Failures() == 10);
}

static void TestCommMonitorRampsFromZeroAfterRecovery() {
  CommMonitor comm(10);
  comm.FrameOk();
  comm.UpdateTorqueScale(100);
  for (int i = 0; i < 11; i++)
    comm.FrameBad();
  comm.FrameOk();
  ASSERT(comm.UpdateTorqueScale(25) == 25 && comm.UpdateTorqueScale(25) == 50);
}

static void TestCommMonitorFailureCountSaturates() {
  CommMonitor comm(10);
  for (long i = 0; i < 70000; i++)
    comm.FrameBad();
  bool saturated = comm.Lost() && comm.Failures() == 65535;
  comm.FrameOk();
  ASSERT(saturated && !comm.Lost());
}

static void TestCommMonitorRampClamps() {
  CommMonitor a(10), b(10);
  a.FrameOk();
  b.FrameOk();
  ASSERT(a.UpdateTorqueScale(30) == 30 && a.UpdateTorqueScale(30) == 60 &&
         a.UpdateTorqueScale(30) == 90 && a.UpdateTorqueScale(30) == 100 &&
         b.UpdateTorqueScale(0) == 100);
}

// OVERSPEED
static void TestOverspeedFullTorqueBelowLimit() {
  OverspeedCut cut;
  ASSERT(cut.Update(9999, 10000, 10) == 100 &&
         cut.Update(-9999, 10000, 10) == 100 && !cut.Cut());
}

static void TestOverspeedCutsAboveLimitBothDirections() {
  OverspeedCut fwd, rev;
  ASSERT(fwd.Update(10001, 10000, 10) == 0 && fwd.Cut() &&
         rev.Update(-10001, 10000, 10) == 0 && rev.Cut());
}

static void TestOverspeedHoldsCutInsideHysteresis() {
  OverspeedCut cut;
  cut.Update(10500, 10000, 10);
  ASSERT(cut.Update(9600, 10000, 10) == 0 && cut.Cut());
}

static void TestOverspeedRampsBackBelowHysteresis() {
  OverspeedCut cut;
  cut.Update(10500, 10000, 10);
  ASSERT(cut.Update(9400, 10000, 10) == 10 && !cut.Cut() &&
         cut.Update(9400, 10000, 10) == 20);
}

static void TestOverspeedHandlesMostNegativeSpeed() {
  OverspeedCut cut;
  ASSERT(cut.Update(-32768, 10000, 10) == 0 && cut.Cut());
}

// HTM BUILDING
static void TestBuildHtmGs450hPlacesTorqueLikeOemFrame() {
  uint8_t htm[GS450H_HTM_LEN];
  memcpy(htm, GS450H_HTM_PEAK_ACCEL, GS450H_HTM_LEN);
  const int torqueBytes[] = {0, 5, 6, 11, 12, 26, 27, 32, 33};
  for (int b : torqueBytes)
    htm[b] = 0;

  // OEM frame: MG1 -160 (sent negated), MG2 1755, speed byte 52
  BuildHtmGs450h(htm, 160, 1755, 0, 52 * 113);

  bool match = true;
  for (int b : torqueBytes)
    match = match && htm[b] == GS450H_HTM_PEAK_ACCEL[b];
  ASSERT(match && Checksum16Ok(htm, GS450H_HTM_LEN));
}

static void TestBuildHtmGs300hPlacesTorqueLikeOemFrame() {
  uint8_t htm[GS300H_HTM_LEN];
  memcpy(htm, PHV_HTM_DRIVE, GS300H_HTM_LEN);
  const int torqueBytes[] = {5, 6, 11, 12, 31, 32};
  for (int b : torqueBytes)
    htm[b] = 0;

  // OEM frame in D: MG1 0, MG2 131
  BuildHtmGs300h(htm, 0, 131);

  bool match = true;
  for (int b : torqueBytes)
    match = match && htm[b] == PHV_HTM_DRIVE[b];
  ASSERT(match && Checksum16Ok(htm, GS300H_HTM_LEN));
}

static void TestBuildHtmPriusStepsCounterEverySecondFrame() {
  uint8_t htm[PRIUS_HTM_LEN];
  memcpy(htm, PRIUS_HTM_BENCH_INIT, PRIUS_HTM_LEN);
  uint8_t frameCount = 0;
  const uint8_t start = htm[94];

  for (int i = 0; i < 4; i++)
    BuildHtmPrius(htm, 0, 0, 0, frameCount, PRIUS_DEFAULT_LIMITS);
  ASSERT(htm[94] == (uint8_t)(start + 2) && frameCount == 4 &&
         Checksum16Ok(htm, PRIUS_HTM_LEN));
}

static void TestBuildHtmPriusDefaultLimitsAreTheOldBytes() {
  uint8_t htm[PRIUS_HTM_LEN];
  memcpy(htm, PRIUS_HTM_BENCH_INIT, PRIUS_HTM_LEN);
  uint8_t frameCount = 0;
  BuildHtmPrius(htm, 0, 0, 0, frameCount, PRIUS_DEFAULT_LIMITS);

  // the hard-coded block and bytes 86/88 sent before the limits became inputs
  const uint8_t old72to79[] = {0x40, 0x04, 0x75, 0x12, 0x60, 0x25, 0x52, 0x04};
  ASSERT(memcmp(&htm[72], old72to79, sizeof(old72to79)) == 0 &&
         htm[86] == 137 && htm[88] == 137);
}

// The Prius test params must default to the values sent so far
static void TestPriusTestParamDefaultsAreTheDefaultLimits() {
  ASSERT(Param::GetInt(Param::PriusRegenLim) ==
             PRIUS_DEFAULT_LIMITS.regenLimit &&
         Param::GetInt(Param::PriusDischLim) ==
             PRIUS_DEFAULT_LIMITS.dischargeLimit &&
         Param::GetInt(Param::PriusByte86) == PRIUS_DEFAULT_LIMITS.byte86 &&
         Param::GetInt(Param::PriusByte88) == PRIUS_DEFAULT_LIMITS.byte88 &&
         Param::GetType(Param::PriusRegenLim) == Param::TYPE_TESTPARAM);
}

static void TestBuildHtmPriusUsesGivenLimits() {
  uint8_t htm[PRIUS_HTM_LEN];
  memcpy(htm, PRIUS_HTM_BENCH_INIT, PRIUS_HTM_LEN);
  uint8_t frameCount = 0;
  const PriusLimits oem = {-2000, 1660, 202, 239};
  BuildHtmPrius(htm, 0, 0, 0, frameCount, oem);

  int16_t regen = htm[76] | (htm[77] << 8);
  int16_t discharge = htm[78] | (htm[79] << 8);
  // bytes 72-75 stay hard-coded
  const uint8_t old72to75[] = {0x40, 0x04, 0x75, 0x12};
  ASSERT(regen == -2000 && discharge == 1660 && htm[86] == 202 &&
         htm[88] == 239 && memcmp(&htm[72], old72to75, 4) == 0 &&
         Checksum16Ok(htm, PRIUS_HTM_LEN));
}

void ToyotaFramesTest::RunTest() {
  TestChecksum16AcceptsAllCapturedFrames();
  TestChecksum16RejectsSingleBitErrors();
  TestChecksum16RejectsShiftedFrames();
  TestChecksum16RejectsTruncatedFrames();
  TestSetChecksum16MatchesCapturedFrames();
  TestDecodeMthGs300hOnPriusPhvFrame();
  TestDecodeMthPriusOnAurisFrames();
  TestReadMthAcceptsValidFrames();
  TestReadMthRejectsCorruptFrameAfterGoodFrame();
  TestReadMthRejectsFrameWithOnly8BitChecksum();
  TestCommMonitorStartsLostWithNoTorque();
  TestCommMonitorRampsTorqueInAfterGoodFrame();
  TestCommMonitorLostAfterMoreThanMaxFailures();
  TestCommMonitorGoodFrameResetsFailureCount();
  TestCommMonitorRampsFromZeroAfterRecovery();
  TestCommMonitorFailureCountSaturates();
  TestCommMonitorRampClamps();
  TestOverspeedFullTorqueBelowLimit();
  TestOverspeedCutsAboveLimitBothDirections();
  TestOverspeedHoldsCutInsideHysteresis();
  TestOverspeedRampsBackBelowHysteresis();
  TestOverspeedHandlesMostNegativeSpeed();
  TestBuildHtmGs450hPlacesTorqueLikeOemFrame();
  TestBuildHtmGs300hPlacesTorqueLikeOemFrame();
  TestBuildHtmPriusStepsCounterEverySecondFrame();
  TestBuildHtmPriusDefaultLimitsAreTheOldBytes();
  TestPriusTestParamDefaultsAreTheDefaultLimits();
  TestBuildHtmPriusUsesGivenLimits();
}
