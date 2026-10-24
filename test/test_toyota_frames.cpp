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
    BuildHtmPrius(htm, 0, 0, 0, frameCount);
  ASSERT(htm[94] == (uint8_t)(start + 2) && frameCount == 4 &&
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
  TestBuildHtmGs450hPlacesTorqueLikeOemFrame();
  TestBuildHtmGs300hPlacesTorqueLikeOemFrame();
  TestBuildHtmPriusStepsCounterEverySecondFrame();
}
