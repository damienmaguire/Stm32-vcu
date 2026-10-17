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
#include "test_list.h"
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

bool DataIs(const uint8_t *data, const uint8_t (&expected)[8]) {
  return data != NULL && memcmp(data, expected, 8) == 0;
}

void Rx(uint32_t id, const uint8_t (&bytes)[8]) {
  uint32_t data[2];
  memcpy(data, bytes, 8);
  charger->DecodeCAN(id, data);
}

// Fresh driver (zero-initialised, as the firmware's static instance is)
void Setup(int opmode, int interface) {
  instance = MGgen2V2Lcharger();
  charger->SetCanInterface(&can);
  Param::SetInt(Param::opmode, opmode);
  Param::SetInt(Param::interface, interface);
  Param::SetInt(Param::Voltspnt, 395);
  Param::SetInt(Param::udc, 380);
  Rx(0x323, {0, 0, 0, 0, 0, 0, 0, 0}); // plug status is file-static
  can.Clear();
}

const uint8_t IDLE_29C[8] = {0x28, 0xFF, 0x83, 0xFF, 0x00, 0xFF, 0x7F, 0xFF};
const uint8_t ACTIVE_29C[8] = {0x28, 0x89, 0x07, 0xFE, 0x00, 0xDC, 0x57, 0x12};

void TestRegistersRxIds() {
  Setup(MOD_OFF, Unused);
  ASSERT(can.IsRegistered(0x324) && can.IsRegistered(0x39F) &&
         can.IsRegistered(0x323));
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

void TestRunSendsActive29CWhenClearToStart() {
  Setup(MOD_RUN, i3LIM);
  charger->ControlCharge(true, true);
  charger->Task100Ms();
  ASSERT(DataIs(can.Find(0x29C), ACTIVE_29C));
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
  ASSERT(DataIs(can.Find(0x29C), ACTIVE_29C));
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

void TestOffOpmodeSendsOnlyIdle29C() {
  Setup(MOD_OFF, Unused);
  charger->Task100Ms();
  const uint32_t ids[] = {0x29C};
  ASSERT(IdsAre(ids, 1));
  ASSERT(DataIs(can.Find(0x29C), IDLE_29C));
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
  // Integer division truncates to whole volts
  ASSERT(Param::GetFloat(Param::U12V) == 14);
  ASSERT(Param::GetInt(Param::I12V) == 20);
}

} // namespace

void MGgen2Test::RunTest() {
  TestRegistersRxIds();
  TestRunFrames();
  TestRunSendsActive29CWhenClearToStart();
  TestChargeFrames();
  TestCharge394DoesNotFollowVoltspntOrUdc();
  TestOffOpmodeSendsOnlyIdle29C();
  TestOffFrames();
  TestOffSendsEvery10thCall();
  TestOffSendsAtOnceAfterRunOrCharge();
  TestControlChargeUnusedFollows323PlugStatus();
  TestControlChargeInterfacesNeedRunAndAcRequest();
  TestDecode39F();
}
