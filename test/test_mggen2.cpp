/*
 * This file is part of the ZombieVerter project.
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

// Host tests for the MG Gen2 OBC/DC-DC driver. A mock CAN interface records
// every frame the driver sends, so the frame set per opmode is pinned and any
// change to it shows up as a test change.

#include "chargers/MGgen2V2Lcharger.h"
#include "iomatrix.h"
#include "test_list.h"
#include <math.h>
#include <string.h>

using namespace std;

namespace {

struct Frame {
  uint32_t id;
  uint8_t len;
  uint8_t data[8];
};

class MockCan : public CanHardware {
public:
  static const int MAX_FRAMES = 64;
  Frame frames[MAX_FRAMES];
  int count = 0;

  void SetBaudrate(enum baudrates) {}
  void Send(uint32_t canId, uint32_t data[2], uint8_t len, bool) {
    if (count < MAX_FRAMES) {
      frames[count].id = canId;
      frames[count].len = len;
      memcpy(frames[count].data, data, 8);
    }
    count++;
  }
  void Clear() { count = 0; }

  bool IsRegistered(uint32_t id) {
    for (int i = 0; i < nextUserMessageIndex; i++)
      if (userIds[i] == id)
        return true;
    return false;
  }

  // Number of frames sent with this ID
  int CountId(uint32_t id) {
    int n = 0;
    for (int i = 0; i < count && i < MAX_FRAMES; i++)
      if (frames[i].id == id)
        n++;
    return n;
  }

  // Data of the nth (0-based) frame sent with this ID, or NULL
  const uint8_t *Find(uint32_t id, int nth = 0) {
    for (int i = 0; i < count && i < MAX_FRAMES; i++)
      if (frames[i].id == id && nth-- == 0)
        return frames[i].data;
    return NULL;
  }

private:
  void ConfigureFilters() {}
};

MockCan can;
MGgen2V2Lcharger instance;
MGgen2V2Lcharger *charger = &instance;

bool IdsAre(const uint32_t *ids, int n) {
  if (can.count != n)
    return false;
  for (int i = 0; i < n; i++)
    if (can.frames[i].id != ids[i] || can.frames[i].len != 8)
      return false;
  return true;
}

bool Near(Param::PARAM_NUM p, float expected) {
  return fabsf(Param::GetFloat(p) - expected) < 0.05f;
}

bool DataIs(const uint8_t *data, const uint8_t (&expected)[8]) {
  return data != NULL && memcmp(data, expected, 8) == 0;
}

void Rx(uint32_t id, const uint8_t (&bytes)[8]) {
  uint32_t data[2];
  memcpy(data, bytes, 8);
  charger->DecodeCAN(id, data);
}

// What the simulated OBC reports in 0x324 (HV volts) and 0x33D (DC amps,
// not sent while negative)
float obcVolts;
float obcDcAmps;

void FeedObc() {
  uint16_t raw = (uint16_t)(obcVolts / 0.02f + 0.5f);
  Rx(0x324,
     {0x20, (uint8_t)(raw >> 8), (uint8_t)(raw & 0xFF), 65, 70, 0, 0, 58});
  if (obcDcAmps >= 0)
    Rx(0x33D,
       {116, 120, 0x50, 0, 0, 0x18, (uint8_t)(obcDcAmps * 10 + 0.5f), 0x20});
}

// Fresh driver (zero-initialised, as the firmware's static instance is)
void Setup(int opmode, int interface) {
  instance = MGgen2V2Lcharger();
  charger->SetCanInterface(&can);
  Param::SetInt(Param::opmode, opmode);
  Param::SetInt(Param::interface, interface);
  Param::SetInt(Param::Voltspnt, 395);
  Param::SetInt(Param::udc, 380);
  Param::SetInt(Param::MGChgMaxA, 20);
  Param::SetInt(Param::BMS_ChargeLim, 9999); // as sent with no BMS
  Param::SetInt(Param::IdcTerm, 0);
  // Plug status and CP duty are file-static: reset to "not plugged"
  Rx(0x323, {0, 0, 0, 0, 0, 0, 0, 0});
  Rx(0x33B, {0, 0, 0, 0, 0, 0, 0, 0});
  obcVolts = 380;
  obcDcAmps = -1;
  FeedObc();
  can.Clear();
}

const uint8_t IDLE_29C[8] = {0x28, 0xFF, 0x83, 0xFF, 0x00, 0xFF, 0x7F, 0xFF};
const uint8_t ACTIVE_29C[8] = {0x28, 0x89, 0x07, 0xFE, 0x00, 0xDC, 0x57, 0x12};

void TestRegistersRxIds() {
  Setup(MOD_OFF, Unused);
  ASSERT(can.IsRegistered(0x324) && can.IsRegistered(0x39F) &&
         can.IsRegistered(0x323) && can.IsRegistered(0x33B) &&
         can.IsRegistered(0x33D));
}

void TestRunFrames() {
  Setup(MOD_RUN, Unused);
  charger->Task100Ms();
  // No 0x29B: b3 = 0x02 there kept the DC-DC off in RUN
  const uint32_t ids[] = {0x19C, 0x297, 0x1F1, 0x33F, 0x29C};
  ASSERT(IdsAre(ids, 5));
  ASSERT(DataIs(can.Find(0x19C), {0x06, 0xA0, 0x26, 0, 0, 0, 0, 0x7F}));
  ASSERT(DataIs(can.Find(0x297), {0, 0x03, 0, 0, 0, 0, 0x20, 0}));
  ASSERT(DataIs(can.Find(0x1F1), {0x0E, 0, 0, 0, 0, 0, 0x20, 0}));
  ASSERT(DataIs(can.Find(0x33F), {0, 0, 0, 0, 0x28, 0, 0, 0x46}));
  ASSERT(DataIs(can.Find(0x29C), IDLE_29C));
}

void TestOnlyChargeSendsActive29C() {
  // Charge current is only permitted in CHARGE, even with a charge request
  const int opmodes[] = {MOD_RUN, MOD_PRECHARGE, MOD_PCHFAIL, MOD_MAINTAIN,
                         MOD_PREHEAT};
  for (int i = 0; i < 5; i++) {
    Setup(opmodes[i], i3LIM);
    charger->ControlCharge(true, true);
    charger->Task100Ms();
    ASSERT(can.CountId(0x29C) == 1 && DataIs(can.Find(0x29C), IDLE_29C));
  }
}

uint16_t LimitRaw(const uint8_t *f) {
  return f == NULL ? 0xFFFF : ((f[2] & 0x0F) << 8) | f[3];
}

// Current limit in the 0x29C frame of the next Task100Ms() tick
uint16_t TickLimit() {
  FeedObc();
  can.Clear();
  charger->Task100Ms();
  return LimitRaw(can.Find(0x29C));
}

// 0x29C once the start ramp has settled (0x428 -> 0x7FE takes 11 ticks)
const uint8_t *Charge29CWithMaxA(float amps) {
  Setup(MOD_CHARGE, i3LIM);
  Param::SetFloat(Param::MGChgMaxA, amps);
  charger->ControlCharge(true, true);
  for (int i = 0; i < 15; i++)
    TickLimit();
  return can.Find(0x29C);
}

void TestChargeStartRamp() {
  // Like the MG5: 0x428 first, then +100 counts (2 A) per 100 ms
  Setup(MOD_CHARGE, i3LIM);
  charger->ControlCharge(true, true);
  uint16_t expected = 0x428;
  for (int i = 0; i < 10; i++) {
    ASSERT(TickLimit() == expected);
    expected += 100;
  }
  ASSERT(TickLimit() == 0x7FE); // 0x428 + 1000 would be 0x810: capped
  ASSERT(TickLimit() == 0x7FE);
}

void TestChargeRampStopsAtTarget() {
  Setup(MOD_CHARGE, i3LIM);
  Param::SetInt(Param::MGChgMaxA, 5); // 0x400 + 250 = 0x4FA
  charger->ControlCharge(true, true);
  ASSERT(TickLimit() == 0x428);
  ASSERT(TickLimit() == 0x48C);
  ASSERT(TickLimit() == 0x4F0);
  ASSERT(TickLimit() == 0x4FA);
  ASSERT(TickLimit() == 0x4FA);

  // A target below the start value is sent straight away
  Setup(MOD_CHARGE, i3LIM);
  Param::SetFloat(Param::MGChgMaxA, 0.5f); // 0x419
  charger->ControlCharge(true, true);
  ASSERT(TickLimit() == 0x419);
}

void TestChargeLimitDecreasesAtOnce() {
  Charge29CWithMaxA(20);
  Param::SetInt(Param::MGChgMaxA, 10);
  ASSERT(TickLimit() == 0x5F4);
  Param::SetInt(Param::MGChgMaxA, 0);
  ASSERT(TickLimit() == 0x400);
  // ... and ramps up again from there
  Param::SetInt(Param::MGChgMaxA, 20);
  ASSERT(TickLimit() == 0x464);
}

void TestChargeRampRestartsEachCharge() {
  Charge29CWithMaxA(20);
  // Charge request dropped and back before the opmode left CHARGE
  obcDcAmps = 0;
  charger->ControlCharge(true, false);
  ASSERT(TickLimit() == 0x400); // stop sequence
  TickLimit();                  // charge complete
  ASSERT(charger->ControlCharge(true, true));
  ASSERT(TickLimit() == 0x428);
  ASSERT(DataIs(can.Find(0x33F), {0, 0, 0, 0, 0x28, 0, 0, 0x41}));

  // Through OFF and back to CHARGE
  Charge29CWithMaxA(20);
  Param::SetInt(Param::opmode, MOD_OFF);
  charger->Task100Ms();
  Param::SetInt(Param::opmode, MOD_CHARGE);
  ASSERT(TickLimit() == 0x428);
}

void TestChargeLimitFromMGChgMaxA() {
  // raw = 0x400 + A / 0.02 (MG5 logs); MGChgMaxA has 1/32 A resolution
  const float amps[] = {0, 0.5f, 10, 15, 19.5f};
  const uint16_t raw[] = {0x400, 0x419, 0x5F4, 0x6EE, 0x7CF};
  for (int i = 0; i < 5; i++) {
    const uint8_t *f = Charge29CWithMaxA(amps[i]);
    ASSERT(LimitRaw(f) == raw[i]);
    // Only b2 bits 0..3 and b3 change; the rest stays as the ZS frame
    ASSERT(f != NULL && f[0] == 0x28 && f[1] == 0x89 && (f[2] & 0xF0) == 0 &&
           f[4] == 0x00 && f[5] == 0xDC && f[6] == 0x57 && f[7] == 0x12);
  }
}

// 0x29C limit once settled with these settings
uint16_t SettledLimit(float maxA, float bmsLimit) {
  Setup(MOD_CHARGE, i3LIM);
  Param::SetFloat(Param::MGChgMaxA, maxA);
  Param::SetFloat(Param::BMS_ChargeLim, bmsLimit);
  charger->ControlCharge(true, true);
  for (int i = 0; i < 15; i++)
    TickLimit();
  return TickLimit();
}

void TestChargeTargetIsMinOfMaxAAndBms() {
  ASSERT(SettledLimit(20, 9999) == 0x7FE); // no BMS
  ASSERT(SettledLimit(20, 1000) == 0x7FE); // 1000 and up = no limit
  ASSERT(SettledLimit(20, 999) == 0x7FE);  // BMS above the OBC maximum
  ASSERT(SettledLimit(20, 10) == 0x5F4);   // BMS limits
  ASSERT(SettledLimit(5, 15) == 0x4FA);    // MGChgMaxA limits
  ASSERT(SettledLimit(20, 0) == 0x400);    // BMS says stop
  ASSERT(SettledLimit(20, -5) == 0x400);
}

void TestBmsLimitDropAppliesAtOnce() {
  Charge29CWithMaxA(20);
  Param::SetInt(Param::BMS_ChargeLim, 8);
  ASSERT(TickLimit() == 0x590);
  Param::SetInt(Param::BMS_ChargeLim, 9999);
  ASSERT(TickLimit() == 0x5F4); // back up at the ramp rate
}

// Raw count for a limit in A, as the driver rounds it
uint16_t Raw(float amps) { return 0x400 + (int)(amps / 0.02f + 0.5f); }

void TestCvTapersAboveVoltspnt() {
  // Voltspnt = 395 V. The taper starts from the current flowing (16 A),
  // not from the 20.4 A limit the OBC never reached
  Charge29CWithMaxA(20);
  obcDcAmps = 16;
  obcVolts = 395.5f; // 0.5 V over: -0.5 A per tick
  ASSERT(TickLimit() == Raw(15.5f));
  ASSERT(TickLimit() == Raw(15));
  obcVolts = 396; // 1 V over: -1 A
  ASSERT(TickLimit() == Raw(14));
  obcVolts = 395.1f; // 0.1 V over: the minimum step, -0.2 A
  ASSERT(TickLimit() == Raw(13.8f));
  obcVolts = 394.5f; // within 1 V below: hold
  ASSERT(TickLimit() == Raw(13.8f));
  ASSERT(TickLimit() == Raw(13.8f));
  obcVolts = 393.9f; // more than 1 V below: +0.1 A
  ASSERT(TickLimit() == Raw(13.9f));
  ASSERT(TickLimit() == Raw(14));
}

void TestCvUsesHigherOfUdcAndObc() {
  // udc above Voltspnt, OBC reading below: the higher one counts
  Charge29CWithMaxA(20);
  Param::SetFloat(Param::udc, 396);
  ASSERT(TickLimit() == Raw(20.44f - 1)); // no 0x33D: tapers from 20.44 A

  // No udc source (0 V while charging): the OBC reading counts
  Charge29CWithMaxA(20);
  Param::SetFloat(Param::udc, 0);
  obcVolts = 396;
  ASSERT(TickLimit() == Raw(20.44f - 1));
}

void TestCvLimitStopsAtZeroAndRecovers() {
  Charge29CWithMaxA(20);
  obcVolts = 399; // 4 V over, below the hard stop
  for (int i = 0; i < 10; i++)
    TickLimit();
  ASSERT(TickLimit() == 0x400);
  obcVolts = 390;
  ASSERT(TickLimit() == Raw(0.1f)); // not a stop: recovers by 0.1 A a tick
}

void TestHardStopAtVoltspntPlus5V() {
  Charge29CWithMaxA(20);
  obcVolts = 399.9f;
  ASSERT(TickLimit() != 0x400);
  Charge29CWithMaxA(20);
  obcVolts = 400.1f;
  ASSERT(TickLimit() == 0x400);
  obcVolts = 380; // latched for the rest of this charge
  for (int i = 0; i < 20; i++)
    TickLimit();
  ASSERT(TickLimit() == 0x400);

  // From udc too
  Charge29CWithMaxA(20);
  Param::SetFloat(Param::udc, 401);
  ASSERT(TickLimit() == 0x400);
}

void TestStopClearsOnNextCharge() {
  Charge29CWithMaxA(20);
  obcVolts = 401;
  obcDcAmps = 0;
  ASSERT(TickLimit() == 0x400);
  obcVolts = 380;
  TickLimit(); // charge complete, latched
  ASSERT(!charger->ControlCharge(true, true));
  ASSERT(!charger->ControlCharge(true, false)); // ACReq false = unplugged
  ASSERT(charger->ControlCharge(true, true));
  ASSERT(TickLimit() == 0x428);
}

// The frames of the last tick are the car's charge complete frames
bool CompleteFramesSent() {
  const uint8_t *f297 = can.Find(0x297), *f29B = can.Find(0x29B);
  const uint8_t *f33F = can.Find(0x33F), *f19C = can.Find(0x19C);
  return DataIs(can.Find(0x29C), {0, 0, 0, 0, 0, 0, 0, 0}) && f297 != NULL &&
         f297[1] == 0x0A && f29B != NULL && f29B[0] == 0xBB &&
         f29B[3] == 0x02 && f33F != NULL && f33F[7] == 0x42 && f19C != NULL &&
         f19C[2] == 0x06;
}

void TestExternalStopWaitsForZeroCurrent() {
  // BMS stop, termination, web UI disable or timer: RunCh goes false
  Charge29CWithMaxA(20);
  obcDcAmps = 16;
  ASSERT(charger->ControlCharge(false, true)); // held in CHARGE
  ASSERT(TickLimit() == 0x400);
  ASSERT(TickLimit() == 0x400); // still 16 A: wait
  ASSERT(charger->ControlCharge(false, true));
  obcDcAmps = 0.8f; // OBC idle plus the DC-DC draw
  TickLimit();
  ASSERT(CompleteFramesSent());
  ASSERT(!charger->ControlCharge(false, true)); // now to OFF
  // Not latched: enabling again restarts the charge
  ASSERT(charger->ControlCharge(true, true));
}

void TestExternalStopTimesOutAfter3s() {
  const float dcAmps[] = {16, -1}; // current stays up / no 0x33D at all
  for (int k = 0; k < 2; k++) {
    Charge29CWithMaxA(20);
    obcDcAmps = dcAmps[k];
    charger->ControlCharge(false, true);
    for (int i = 0; i < 30; i++)
      ASSERT(TickLimit() == 0x400);
    TickLimit(); // 3 s
    ASSERT(CompleteFramesSent());
  }
}

void TestCompleteFramesRepeatUntilOff() {
  Charge29CWithMaxA(20);
  obcDcAmps = 0;
  charger->ControlCharge(false, true);
  TickLimit();
  TickLimit();
  ASSERT(CompleteFramesSent());
  TickLimit(); // still in CHARGE: the car keeps sending them
  ASSERT(CompleteFramesSent());
  Param::SetInt(Param::opmode, MOD_OFF);
  charger->Task100Ms();
  Param::SetInt(Param::opmode, MOD_CHARGE);
  charger->ControlCharge(true, true);
  ASSERT(TickLimit() == 0x428);
  ASSERT(!CompleteFramesSent());
  const uint8_t *f19C = can.Find(0x19C);
  ASSERT(f19C != NULL && f19C[2] == 0x26); // DC-DC on again
}

void TestStopBeforeChargingIsImmediate() {
  Setup(MOD_PRECHARGE, i3LIM);
  ASSERT(charger->ControlCharge(true, true));
  ASSERT(!charger->ControlCharge(false, true));
}

void TestCvPhaseCompletesAfter30s() {
  Charge29CWithMaxA(20);
  obcVolts = 399; // 4 V over Voltspnt: the CV limit falls to 0 at once
  obcDcAmps = 0.5f;
  int ticks = 0;
  do {
    TickLimit();
    ticks++;
  } while (!CompleteFramesSent() && ticks < 400);
  // 300 ticks at the CV end, one at 0 A, then charge complete
  ASSERT(ticks == 301);

  // Latched while still plugged in, cleared by unplugging (ACReq false)
  obcVolts = 380;
  ASSERT(!charger->ControlCharge(true, true));
  charger->ControlCharge(true, false);
  ASSERT(charger->ControlCharge(true, true));
}

// Ticks until charge complete with the CV limit held at 2 A
int TicksToCompleteAt2A(int idcTerm) {
  Charge29CWithMaxA(20);
  Param::SetInt(Param::IdcTerm, idcTerm);
  obcDcAmps = 16;
  obcVolts = 396; // 1 V over: 16 A -> 2 A in 14 ticks
  for (int i = 0; i < 14; i++)
    TickLimit();
  obcVolts = 394.5f; // hold band: the limit stays at 2 A
  obcDcAmps = 0.5f;  // the OBC follows the falling limit
  int ticks = 0;
  do {
    TickLimit();
    ticks++;
  } while (!CompleteFramesSent() && ticks < 400);
  return ticks;
}

void TestCvTerminationUsesIdcTerm() {
  ASSERT(TicksToCompleteAt2A(0) == 400); // 2 A > 1 A floor: keeps charging
  ASSERT(TicksToCompleteAt2A(3) < 300);  // 2 A <= IdcTerm 3 A: complete
}

void TestHardStopLatchUnusedClearsOnUnplug() {
  Setup(MOD_CHARGE, Unused);
  Rx(0x33B, {50, 0, 0, 0, 0, 0, 0, 0});
  ASSERT(charger->ControlCharge(true, true));
  obcDcAmps = 0;
  obcVolts = 401;
  TickLimit(); // hard stop: 0 A
  Rx(0x33B, {50, 0, 0, 0, 0, 0, 0, 0});
  TickLimit();
  ASSERT(CompleteFramesSent());
  obcVolts = 380;
  Rx(0x33B, {50, 0, 0, 0, 0, 0, 0, 0});
  ASSERT(!charger->ControlCharge(true, true)); // still plugged in
  Rx(0x33B, {100, 0, 0, 0, 0, 0, 0, 0});       // EVSE stopped PWM
  ASSERT(!charger->ControlCharge(true, true));
  Rx(0x33B, {50, 0, 0, 0, 0, 0, 0, 0});
  ASSERT(charger->ControlCharge(true, true));
}

void TestDoneLatchClearsInRun() {
  Charge29CWithMaxA(20);
  obcDcAmps = 0;
  obcVolts = 401;
  TickLimit();
  TickLimit();
  ASSERT(!charger->ControlCharge(true, true));
  Param::SetInt(Param::opmode, MOD_RUN);
  charger->Task100Ms();
  ASSERT(charger->ControlCharge(true, true));
}

void TestLostObcVoltageStops() {
  // udc may come from an inverter that is off while charging, so without
  // 0x324 an overvoltage could go unseen
  Charge29CWithMaxA(20);        // its last tick is the 1st after the last 0x324
  for (int i = 0; i < 9; i++) { // up to 1 s without 0x324: still fresh
    can.Clear();
    charger->Task100Ms();
  }
  ASSERT(LimitRaw(can.Find(0x29C)) == 0x7FE);
  can.Clear();
  charger->Task100Ms();
  ASSERT(LimitRaw(can.Find(0x29C)) == 0x400);
  ASSERT(TickLimit() == 0x400); // stays stopped when 0x324 is back
}

void TestChargeLimitTopOfRangeIsUnrestricted() {
  // 20 A (the default) sends 0x7FE, the same frame as before
  ASSERT(DataIs(Charge29CWithMaxA(20), ACTIVE_29C));
}

void TestChargeLimitIsClamped() {
  // SetFloat bypasses the web UI range check; the driver clamps anyway
  ASSERT(DataIs(Charge29CWithMaxA(25), ACTIVE_29C));
  ASSERT(LimitRaw(Charge29CWithMaxA(-1)) == 0x400);
}

void TestMGChgMaxAOnlyAffectsCharge() {
  Setup(MOD_RUN, i3LIM);
  Param::SetInt(Param::MGChgMaxA, 5);
  charger->ControlCharge(true, true);
  charger->Task100Ms();
  ASSERT(DataIs(can.Find(0x29C), IDLE_29C));
}

void TestChargeWithoutClearToStartSendsIdle29C() {
  Setup(MOD_CHARGE, i3LIM);
  charger->ControlCharge(true, false);
  charger->Task100Ms();
  ASSERT(can.CountId(0x29C) == 1 && DataIs(can.Find(0x29C), IDLE_29C));
}

void TestChargeFrames() {
  Setup(MOD_CHARGE, i3LIM);
  charger->ControlCharge(true, true);
  charger->Task100Ms();
  const uint32_t ids[] = {0x1F1, 0x297, 0x29B, 0x32E, 0x343, 0x348, 0x396,
                          0x39A, 0x39B, 0x33F, 0x19C, 0x322, 0x394, 0x29C};
  ASSERT(IdsAre(ids, 14));
  ASSERT(DataIs(can.Find(0x1F1), {0x0E, 0, 0, 0, 0, 0, 0x20, 0}));
  ASSERT(DataIs(can.Find(0x297), {0, 0x06, 0, 0, 0, 0x20, 0x20, 0}));
  ASSERT(DataIs(can.Find(0x29B), {0x3B, 0xCA, 0x86, 0x00, 0xFD, 0x60, 0, 0}));
  ASSERT(DataIs(can.Find(0x32E), {0, 0, 0, 0, 0, 0, 0, 0}));
  ASSERT(DataIs(can.Find(0x343), {0, 0, 0, 0, 0, 0, 0, 0}));
  ASSERT(DataIs(can.Find(0x348), {0, 0, 0, 0, 0, 0, 0, 0}));
  ASSERT(DataIs(can.Find(0x396),
                {0x44, 0x6E, 0xB4, 0x28, 0x80, 0x4E, 0x4E, 0x4D}));
  ASSERT(DataIs(can.Find(0x39A), {0, 0x43, 0, 0, 0xCD, 0, 0, 0}));
  ASSERT(DataIs(can.Find(0x39B), {0x44, 0x43, 0x9D, 0, 0, 0, 0, 0}));
  ASSERT(DataIs(can.Find(0x33F), {0, 0, 0, 0, 0x28, 0, 0, 0x41}));
  ASSERT(DataIs(can.Find(0x19C), {0x06, 0xA0, 0x26, 0, 0, 0, 0, 0x7F}));
  ASSERT(DataIs(can.Find(0x322), {0, 0, 0, 0x20, 0, 0, 0, 0}));
  ASSERT(DataIs(can.Find(0x394), {0, 0x28, 0, 0, 0x10, 0x43, 0, 0}));
  // First tick of a charge: the ramp starts at 0x428 (0.8 A)
  ASSERT(
      DataIs(can.Find(0x29C), {0x28, 0x89, 0x04, 0x28, 0, 0xDC, 0x57, 0x12}));
}

void TestCharge394DoesNotFollowVoltspntOrUdc() {
  const uint8_t car394[8] = {0, 0x28, 0, 0, 0x10, 0x43, 0, 0};
  const int udcs[] = {380, 395, 400};
  for (int i = 0; i < 3; i++) {
    Setup(MOD_CHARGE, i3LIM);
    Param::SetInt(Param::Voltspnt, 398);
    Param::SetInt(Param::udc, udcs[i]);
    charger->ControlCharge(true, true);
    for (int tick = 0; tick < 3; tick++) {
      can.Clear();
      charger->Task100Ms();
      ASSERT(can.CountId(0x394) == 1 && DataIs(can.Find(0x394), car394));
    }
  }
}

void TestOffOpmodeSendsNothing() {
  // Off() covers the shutdown delay; after that the bus stays quiet
  Setup(MOD_OFF, Unused);
  charger->Task100Ms();
  ASSERT(can.count == 0);

  Setup(MOD_OFF, i3LIM);
  charger->ControlCharge(true, true);
  charger->Task100Ms();
  ASSERT(can.count == 0);
}

void TestOffFrames() {
  Setup(MOD_OFF, Unused);
  charger->Off();
  const uint32_t ids[] = {0x19C, 0x297, 0x29C, 0x1F1, 0x33F, 0x322,
                          0x39B, 0x39A, 0x396, 0x343, 0x32E, 0x29B};
  ASSERT(IdsAre(ids, 12));
  ASSERT(DataIs(can.Find(0x19C), {0x26, 0xA0, 0x06, 0, 0, 0, 0, 0x7F}));
  ASSERT(DataIs(can.Find(0x297), {0, 0x01, 0, 0, 0, 0, 0, 0}));
  ASSERT(DataIs(can.Find(0x29C), IDLE_29C));
  ASSERT(DataIs(can.Find(0x1F1), {0, 0, 0, 0, 0, 0, 0x20, 0}));
  // One 0x33F with the off request, no longer toggling 00/41
  ASSERT(DataIs(can.Find(0x33F), {0, 0, 0, 0, 0x28, 0, 0, 0x00}));
  ASSERT(DataIs(can.Find(0x29B), {0x3B, 0xCA, 0x86, 0x00, 0xFD, 0x60, 0, 0}));
}

void TestOffSendsEvery10thCall() {
  // Off() is called from the 10 ms task during the 2.5 s shutdown window
  Setup(MOD_OFF, Unused);
  charger->Off();
  ASSERT(can.count == 12);
  for (int i = 1; i < 10; i++)
    charger->Off();
  ASSERT(can.count == 12);
  charger->Off();
  ASSERT(can.count == 24);

  Setup(MOD_OFF, Unused);
  for (int i = 0; i < 250; i++)
    charger->Off();
  ASSERT(can.count == 25 * 12);
}

void TestOffSendsAtOnceAfterRunOrCharge() {
  const int opmodes[] = {MOD_RUN, MOD_CHARGE};
  for (int i = 0; i < 2; i++) {
    Setup(MOD_OFF, Unused);
    for (int j = 0; j < 3; j++) // ends part way through a 100 ms period
      charger->Off();
    Param::SetInt(Param::opmode, opmodes[i]);
    charger->Task100Ms();
    Param::SetInt(Param::opmode, MOD_OFF);
    can.Clear();
    charger->Off();
    ASSERT(can.count == 12);
  }
}

void TestControlChargeUnusedFollows323PlugStatus() {
  Setup(MOD_OFF, Unused);
  ASSERT(!charger->ControlCharge(true, true));
  Rx(0x323, {0, 0, 0, 0, 0, 0x01, 0, 0});
  ASSERT(charger->ControlCharge(true, true));
  ASSERT(!charger->ControlCharge(true, false));
  Rx(0x323, {0, 0, 0, 0, 0, 0x00, 0, 0});
  ASSERT(!charger->ControlCharge(true, true));
}

void TestControlChargeUnusedFollowsCpDuty() {
  Setup(MOD_OFF, Unused);
  const uint8_t duty[] = {0, 1, 9, 50, 99, 100, 0x80 | 50, 0x80 | 100};
  const bool plugged[] = {false, true, true, true, true, false, true, false};
  for (int i = 0; i < 8; i++) {
    Rx(0x33B, {duty[i], 0, 0, 0, 0, 0, 0, 0});
    ASSERT(charger->ControlCharge(true, true) == plugged[i]);
  }
}

void TestControlChargeUnusedNeedsRunCh() {
  // RunCh false = charge terminated, BMS stop, web UI disable or timer
  Setup(MOD_OFF, Unused);
  Rx(0x33B, {50, 0, 0, 0, 0, 0, 0, 0});
  ASSERT(!charger->ControlCharge(false, true));
  Setup(MOD_OFF, Unused);
  Rx(0x323, {0, 0, 0, 0, 0, 0x01, 0, 0});
  ASSERT(!charger->ControlCharge(false, true));
}

void TestPlugDetectTimesOutAfter1s() {
  Setup(MOD_OFF, Unused);
  Rx(0x33B, {50, 0, 0, 0, 0, 0, 0, 0});
  for (int i = 0; i < 10; i++)
    charger->Task100Ms();
  ASSERT(charger->ControlCharge(true, true));
  charger->Task100Ms();
  ASSERT(!charger->ControlCharge(true, true));

  Setup(MOD_OFF, Unused);
  Rx(0x323, {0, 0, 0, 0, 0, 0x01, 0, 0});
  for (int i = 0; i < 10; i++)
    charger->Task100Ms();
  ASSERT(charger->ControlCharge(true, true));
  charger->Task100Ms();
  ASSERT(!charger->ControlCharge(true, true));
}

void TestControlChargeInterfacesNeedRunAndAcRequest() {
  const int interfaces[] = {i3LIM, CPC, Foccci, Chademo};
  for (int i = 0; i < 4; i++) {
    Setup(MOD_OFF, interfaces[i]);
    ASSERT(charger->ControlCharge(true, true) &&
           !charger->ControlCharge(false, true) &&
           !charger->ControlCharge(true, false));
  }
}

void TestDecode39F() {
  Setup(MOD_RUN, Unused);
  Rx(0x39F, {0, 113, 0, 0, 20, 0, 0, 0}); // 113 * 0.125 = 14.125 V
  ASSERT(Near(Param::U12V, 14.125f));
  ASSERT(Param::GetInt(Param::I12V) == 20);
}

void TestDecode33DPublishesAc() {
  Setup(MOD_CHARGE, Unused);
  Rx(0x33D, {116, 120, 0x50, 0, 0, 0x18, 161, 0x20}); // Ben, 6 kW log
  ASSERT(Near(Param::AC_Amps, 23.2f));
  ASSERT(Near(Param::AC_Volts, 240));
}

void TestDecode324PublishesChgTemp() {
  Setup(MOD_CHARGE, Unused);
  Rx(0x324, {0x20, 0x48, 0x8A, 65, 70, 0, 0, 58}); // 371.4 V, 25/30 degC
  ASSERT(Near(Param::ChgTemp, 30));
}

void TestPilotLimFromCpDuty() {
  Setup(MOD_OFF, Unused);
  const uint8_t duty[] = {0, 9, 10, 16, 26, 50, 85, 90, 96, 97, 100};
  const float amps[] = {0, 0, 6, 9.6f, 15.6f, 30, 51, 65, 80, 0, 0};
  for (int i = 0; i < 11; i++) {
    Rx(0x33B, {duty[i], 0, 0, 0, 0, 0, 0, 0});
    ASSERT(Near(Param::PilotLim, amps[i]));
  }
}

void TestPlugDetFollowsPlug() {
  Setup(MOD_OFF, Unused);
  Rx(0x33B, {50, 0, 0, 0, 0, 0, 0, 0});
  ASSERT(Param::GetInt(Param::PlugDet) == 1);
  Rx(0x33B, {100, 0, 0, 0, 0, 0, 0, 0});
  ASSERT(Param::GetInt(Param::PlugDet) == 0);
  Rx(0x323, {0, 0, 0, 0, 0, 0x01, 0, 0});
  ASSERT(Param::GetInt(Param::PlugDet) == 1);
  Rx(0x323, {0, 0, 0, 0, 0, 0x00, 0, 0});
  ASSERT(Param::GetInt(Param::PlugDet) == 0);
}

void TestChargeInterfaceKeepsPilotLimAndPlugDet() {
  // i3LIM, CPC and Foccci write these themselves
  Setup(MOD_OFF, i3LIM);
  Param::SetInt(Param::PilotLim, 7);
  Param::SetInt(Param::PlugDet, 0);
  Rx(0x33B, {50, 0, 0, 0, 0, 0, 0, 0});
  Rx(0x323, {0, 0, 0, 0, 0, 0x01, 0, 0});
  ASSERT(Param::GetInt(Param::PilotLim) == 7);
  ASSERT(Param::GetInt(Param::PlugDet) == 0);
}

void TestPilotProxPinKeepsPlugDet() {
  // stm32_vcu.cpp writes PlugDet from the PP input
  Setup(MOD_OFF, Unused);
  Param::SetInt(Param::GPA2Func, IOMatrix::PILOT_PROX);
  Param::SetInt(Param::PlugDet, 0);
  Rx(0x33B, {50, 0, 0, 0, 0, 0, 0, 0});
  ASSERT(Param::GetInt(Param::PlugDet) == 0);
  ASSERT(Near(Param::PilotLim, 30));
  Param::SetInt(Param::GPA2Func, IOMatrix::NONE_ANAL);
}

void TestPublishesChargerDcValues() {
  Setup(MOD_CHARGE, i3LIM);
  obcVolts = 371.4f; // Ben, 6 kW log
  obcDcAmps = 16.1f;
  FeedObc();
  ASSERT(Near(Param::ChgHVVolts, 371.4f));
  ASSERT(Near(Param::ChgDCAmps, 16.1f));
}

void TestPublishesCommandedLimit() {
  Setup(MOD_OFF, i3LIM);
  charger->Task100Ms();
  ASSERT(Near(Param::ChgLimAmps, 0)); // idle frame

  Setup(MOD_CHARGE, i3LIM);
  charger->ControlCharge(true, true);
  TickLimit();
  ASSERT(Near(Param::ChgLimAmps, 0.8f)); // 0x428, start of the ramp
  for (int i = 0; i < 15; i++)
    TickLimit();
  ASSERT(Near(Param::ChgLimAmps, 20.44f)); // 0x7FE

  obcDcAmps = 0;
  charger->ControlCharge(false, true);
  TickLimit();
  ASSERT(Near(Param::ChgLimAmps, 0)); // 0x400 while stopping
  TickLimit();
  ASSERT(Near(Param::ChgLimAmps, 0)); // charge complete
}

void TestChargerDcValuesZeroOnRxTimeout() {
  Setup(MOD_OFF, i3LIM);
  obcDcAmps = 16.1f;
  FeedObc();
  for (int i = 0; i < 11; i++)
    charger->Task100Ms();
  ASSERT(Near(Param::ChgHVVolts, 0) && Near(Param::ChgDCAmps, 0));
}

void TestValuesZeroOnceOnRxTimeout() {
  Setup(MOD_OFF, Unused);
  Rx(0x33B, {50, 0, 0, 0, 0, 0, 0, 0});
  Rx(0x33D, {116, 120, 0x50, 0, 0, 0x18, 161, 0x20});
  Rx(0x324, {0x20, 0x48, 0x8A, 65, 70, 0, 0, 58});
  Rx(0x39F, {0, 113, 0, 0, 20, 0, 0, 0});
  for (int i = 0; i < 10; i++)
    charger->Task100Ms();
  ASSERT(Param::GetInt(Param::PlugDet) == 1 && Near(Param::PilotLim, 30) &&
         Near(Param::AC_Volts, 240) && Near(Param::AC_Amps, 23.2f) &&
         Near(Param::ChgTemp, 30) && Near(Param::U12V, 14.125f) &&
         Param::GetInt(Param::I12V) == 20);

  charger->Task100Ms(); // 1.1 s without frames
  ASSERT(Param::GetInt(Param::PlugDet) == 0 && Near(Param::PilotLim, 0) &&
         Near(Param::AC_Volts, 0) && Near(Param::AC_Amps, 0) &&
         Near(Param::ChgTemp, 0) && Near(Param::U12V, 0) &&
         Param::GetInt(Param::I12V) == 0);

  // Zeroed once only, so another source of U12V is not overwritten
  Param::SetFloat(Param::U12V, 12.5f);
  charger->Task100Ms();
  ASSERT(Near(Param::U12V, 12.5f));
}

} // namespace

void MGgen2Test::RunTest() {
  TestRegistersRxIds();
  TestRunFrames();
  TestOnlyChargeSendsActive29C();
  TestChargeWithoutClearToStartSendsIdle29C();
  TestChargeStartRamp();
  TestChargeRampStopsAtTarget();
  TestChargeLimitDecreasesAtOnce();
  TestChargeRampRestartsEachCharge();
  TestChargeLimitFromMGChgMaxA();
  TestChargeLimitTopOfRangeIsUnrestricted();
  TestChargeLimitIsClamped();
  TestChargeTargetIsMinOfMaxAAndBms();
  TestBmsLimitDropAppliesAtOnce();
  TestCvTapersAboveVoltspnt();
  TestCvUsesHigherOfUdcAndObc();
  TestCvLimitStopsAtZeroAndRecovers();
  TestHardStopAtVoltspntPlus5V();
  TestStopClearsOnNextCharge();
  TestLostObcVoltageStops();
  TestExternalStopWaitsForZeroCurrent();
  TestExternalStopTimesOutAfter3s();
  TestCompleteFramesRepeatUntilOff();
  TestStopBeforeChargingIsImmediate();
  TestCvPhaseCompletesAfter30s();
  TestCvTerminationUsesIdcTerm();
  TestHardStopLatchUnusedClearsOnUnplug();
  TestDoneLatchClearsInRun();
  TestMGChgMaxAOnlyAffectsCharge();
  TestChargeFrames();
  TestCharge394DoesNotFollowVoltspntOrUdc();
  TestOffOpmodeSendsNothing();
  TestOffFrames();
  TestOffSendsEvery10thCall();
  TestOffSendsAtOnceAfterRunOrCharge();
  TestControlChargeUnusedFollows323PlugStatus();
  TestControlChargeUnusedFollowsCpDuty();
  TestControlChargeUnusedNeedsRunCh();
  TestPlugDetectTimesOutAfter1s();
  TestControlChargeInterfacesNeedRunAndAcRequest();
  TestDecode39F();
  TestDecode33DPublishesAc();
  TestDecode324PublishesChgTemp();
  TestPilotLimFromCpDuty();
  TestPlugDetFollowsPlug();
  TestChargeInterfaceKeepsPilotLimAndPlugDet();
  TestPilotProxPinKeepsPlugDet();
  TestValuesZeroOnceOnRxTimeout();
  TestPublishesChargerDcValues();
  TestPublishesCommandedLimit();
  TestChargerDcValuesZeroOnRxTimeout();
}
