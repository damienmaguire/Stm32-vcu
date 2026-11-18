// Host check for the E92 MSD80 checksum, alive counter and the worked
// examples in the firmware notes.
//
//   g++ -std=c++17 -Wall -Wextra -pedantic -I../../include -o e92_check check.cpp && ./e92_check
//
// The analysis verified byte0 = ((s >> 8) + (s & 0xFF)) & 0xFF, with
// s = CAN id + the other data bytes, on every logged 0x0A8 / 0x0A9 /
// 0x0AA frame, and the same sum with the checksum in byte 2 on 0x2F3.
// Alive counters step 0..14. The literals below are the idle and
// engine-off pictures described there (torque +17 / -114, state 8 / F,
// rpm at 0.25 rpm per bit, coolant byte - 48, 0xC6 = 994 hPa).

#include "vehicles/BMW_E92_codec.h"

#include <cstdio>
#include <cstdlib>

static int fails = 0;

static void expect(bool ok, const char *what) {
  if (ok)
    return;
  std::printf("FAIL %s\n", what);
  fails++;
}

// Independent copy of the documented fold, so a bug shared with the
// codec still has to hit the literals further down.
static uint8_t refChecksum(uint16_t id, const uint8_t *data, int dlc,
                           int skip) {
  unsigned s = id;
  for (int i = 0; i < dlc; i++) {
    if (i == skip)
      continue;
    s += data[i];
  }
  return (uint8_t)(((s >> 8) + (s & 0xFFu)) & 0xFFu);
}

static int read12(const uint8_t *b, int lsb) {
  uint16_t u = 0;
  for (int i = 0; i < 12; i++) {
    int bit = lsb + i;
    if (b[bit >> 3] & (1u << (bit & 7)))
      u = (uint16_t)(u | (1u << i));
  }
  if (u & 0x800u)
    return (int)u - 4096;
  return (int)u;
}

int main() {
  // Idle: 800 rpm, pedal up, torque +17, state 8, alive 0.
  // Hand sum: other bytes 0x10 01 00 80 0C 80 00, id 0x0AA.
  // s = 0x1C7, fold = 0xC8.
  E92Engine idle;
  idle.rpm = 800;
  idle.pedal = 0;
  idle.state = 8;
  idle.torque = 17;
  uint8_t aa[8];
  E92Build0AA(aa, 0, idle);
  expect(aa[0] == 0xC8, "idle 0x0AA checksum literal 0xC8");
  expect(aa[0] == refChecksum(0x0AA, aa, 8, 0), "idle checksum vs ref");
  expect(aa[0] == E92Checksum(0x0AA, aa, 8, 0), "idle checksum vs codec");
  expect((aa[1] & 0x0F) == 0, "idle alive nibble 0");
  expect(read12(aa, 12) == 17, "idle torque +17");
  expect(aa[3] == 0, "idle pedal 0");
  expect(aa[4] == 0x80 && aa[5] == 0x0C, "800 rpm -> 0x0C80");
  expect((aa[6] >> 4) == 8, "idle state 8");

  // Engine off: torque -114, rpm 0, state 0xF.
  // Other bytes 0xE0 F8 00 00 00 F0 00, id 0x0AA. s = 0x372, fold = 0x75.
  E92Engine off;
  off.rpm = 0;
  off.pedal = 0;
  off.state = 0xF;
  off.torque = -114;
  E92Build0AA(aa, 0, off);
  expect(aa[0] == 0x75, "engine-off checksum literal 0x75");
  expect(aa[0] == refChecksum(0x0AA, aa, 8, 0), "off checksum vs ref");
  expect(read12(aa, 12) == -114, "engine-off torque -114");
  expect(aa[4] == 0 && aa[5] == 0, "engine-off rpm 0");
  expect(aa[3] == 0, "engine-off pedal 0");
  expect((aa[6] >> 4) == 0xF, "engine-off state F");

  // Alive 7 must not disturb the -114 torque nibble (0xE).
  E92Build0AA(aa, 7, off);
  expect((aa[1] & 0x0F) == 7, "alive 7 in the low nibble");
  expect(read12(aa, 12) == -114, "torque intact beside alive 7");
  expect(aa[0] == refChecksum(0x0AA, aa, 8, 0), "alive 7 checksum");

  // A non-zero pedal forces state 9, which is the log rule.
  E92Engine pushed = idle;
  pushed.pedal = 127;
  pushed.state = 8;
  E92Build0AA(aa, 1, pushed);
  expect(aa[3] == 127, "pedal byte");
  expect((aa[6] >> 4) == 9, "pedal forces state 9");
  expect((aa[1] & 0x0F) == 1, "alive 1");
  expect(aa[0] == refChecksum(0x0AA, aa, 8, 0), "pedal frame checksum");

  // State 9 with a zero pedal is rewritten to idle. The logs have no
  // exceptions to that pairing.
  E92Engine bare = idle;
  bare.state = 9;
  bare.pedal = 0;
  E92Build0AA(aa, 0, bare);
  expect((aa[6] >> 4) == 8 && aa[3] == 0, "state 9 without pedal becomes 8");

  // Counters: 0..14 then 0. Never 15.
  uint8_t alive = 0;
  for (int n = 0; n < 30; n++) {
    E92Build0AA(aa, alive, idle);
    uint8_t got = (uint8_t)(aa[1] & 0x0F);
    expect(got == (uint8_t)(n % 15), "alive steps 0..14");
    expect(got != 15, "alive is never 15");
    expect(aa[0] == refChecksum(0x0AA, aa, 8, 0), "each alive checksum");
    alive = E92NextAlive(alive);
  }
  expect(E92NextAlive(14) == 0, "14 wraps to 0");
  expect(E92NextAlive(15) == 0, "15 is not a sent value");

  // 0x0A8 carries the same torque twice and the path-status byte.
  uint8_t a8[8];
  E92Build0A8(a8, 1, 17);
  expect(read12(a8, 12) == 17 && read12(a8, 28) == 17, "0x0A8 both torques");
  expect((a8[1] & 0x0F) == 1, "0x0A8 alive");
  expect(a8[6] == 0xCF, "0x0A8 path status is constant 0xCF");
  expect(a8[0] == refChecksum(0x0A8, a8, 8, 0), "0x0A8 checksum");
  expect(a8[0] == E92Checksum(0x0A8, a8, 8, 0), "0x0A8 codec checksum");

  // 0x0A9: four torques, alive in byte 1, its own counter.
  uint8_t a9[8];
  E92Build0A9(a9, 14, -114);
  expect((a9[1] & 0x0F) == 14, "0x0A9 alive 14");
  expect(read12(a9, 16) == -114 && read12(a9, 28) == -114, "0x0A9 torques 1");
  expect(read12(a9, 40) == -114 && read12(a9, 52) == -114, "0x0A9 torques 2");
  expect(a9[0] == refChecksum(0x0A9, a9, 8, 0), "0x0A9 checksum");

  // 0x2F3 keeps the checksum in byte 2. Alive 3, byte 1 is the
  // logged 0xFC. s = 0x2F3 + 0x03 + 0xFC = 0x3F2, fold = 0xF5.
  uint8_t f3[3];
  E92Build2F3(f3, 3);
  expect(f3[0] == 0x03, "0x2F3 alive in byte 0");
  expect(f3[1] == 0xFC, "0x2F3 byte 1 is the logged constant");
  expect(f3[2] == 0xF5, "0x2F3 checksum literal 0xF5");
  expect(f3[2] == refChecksum(0x2F3, f3, 3, 2), "0x2F3 checksum vs ref");
  E92Build2F3(f3, 12);
  expect(f3[0] == 0x0C && f3[1] == 0xFC && f3[2] == 0xFE,
         "logged idle 0x2F3 is 0C FC FE");

  // The written formula masks once. A full second fold would turn
  // 0x106 into 0x07. Logged frames never reach this sum; the literal
  // pins the expression the notes give.
  uint8_t big[8] = {0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  expect(E92Checksum(0x7FF, big, 8, 0) == 0x06, "single fold then mask");
  expect(refChecksum(0x7FF, big, 8, 0) == 0x06, "ref single fold");

  // Signal pictures, same policy as the E39 tach floor.
  E92Engine pic = E92EnginePicture(100, 0.0f, true);
  expect(pic.rpm == 750 && pic.state == 8 && pic.pedal == 0 && pic.torque == 16,
         "ignition on, pedal up, rpm floored at 750");
  pic = E92EnginePicture(-2000, 50.0f, true);
  expect(pic.rpm == 2000 && pic.state == 9 && pic.pedal == 127 &&
             pic.torque == 164,
         "50% pedal, reverse rpm taken as positive");
  pic = E92EnginePicture(3000, 40.0f, false);
  expect(pic.rpm == 0 && pic.state == 0xF && pic.pedal == 0 &&
             pic.torque == -114,
         "ignition off is the engine-off picture");
  pic = E92EnginePicture(400, -100.0f, true);
  expect(pic.state == 8 && pic.pedal == 0 && pic.rpm == 750 && pic.torque == -68,
         "full regen is overrun -68, not a pedal");
  expect(E92TorqueRaw(true, 0.0f) == 16, "idle torque +16");
  expect(E92TorqueRaw(true, -30.0f) == -49, "part overrun");
  expect(E92TorqueRaw(true, 100.0f) == 312, "full pedal is +312 counts");
  expect(E92NmToCounts(156.0f) == 312, "0.5 Nm/bit: 156 Nm -> 312");
  expect(E92NmToCounts(2000.0f) == 2047, "torque clamps at +2047");
  expect(E92NmToCounts(-2000.0f) == -2048, "torque clamps at -2048");

  expect(E92CoolantByte(90.0f) == 138, "90 C -> byte 138");
  expect(E92CoolantByte(-80.0f) == 0, "cold clamp");
  expect(E92CoolantByte(300.0f) == 254, "0xFF stays the invalid code");
  expect(E92AmbientHpa(0xC6) == 994, "0xC6 is 994 hPa");

  uint8_t bat[8];
  E92Build3B4(bat, 13.8f);
  expect(bat[0] == 0x98 && (bat[1] & 0x0F) == 0x03, "13.8 V -> raw 920");
  expect(bat[1] == 0xF3 && bat[2] == 0x00 && bat[3] == 0xFC, "0x3B4 logged tail");
  expect(bat[4] == 0xFF && bat[5] == 0xFF && bat[6] == 0xFF && bat[7] == 0xFF,
         "0x3B4 bytes 4-7 are FF");
  expect(E92BatteryRaw(0.0f) == 920, "unwired 12 V pin sends 13.8 V");
  // Logged idle frame C8 F3 is 968 counts. 968 * 0.015 = 14.52 V,
  // which the notes round to 14.5 V. The constant bytes around it match.
  const uint8_t logged3B4[8] = {0xC8, 0xF3, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF};
  uint16_t loggedRaw =
      (uint16_t)(logged3B4[0] | ((logged3B4[1] & 0x0F) << 8));
  expect(loggedRaw == 0x3C8, "logged 0x3B4 is 968 counts");
  expect(logged3B4[1] == 0xF3 && logged3B4[3] == 0xFC, "logged 0x3B4 constants");

  uint8_t d0[8];
  E92Build1D0(d0, 4, 90.0f, 0x3C, 0xC6, 0x1234);
  expect(d0[0] == 138 && d0[1] == 0x3C, "coolant stays off the air byte");
  expect(d0[1] != d0[0], "byte 1 is not a copy of coolant");
  expect(d0[2] == 4, "0x1D0 alive in byte 2");
  expect(d0[3] == 0xC6, "ambient byte");
  expect(d0[4] == 0x34 && d0[5] == 0x12, "fuel counter little-endian");
  E92Build1D0(d0, 0, 20.0f, 0x08, 0xC6, 0);
  expect(d0[1] == 0x08, "air byte is 0x08 before running");

  // Logged idle frames. The checksum byte has to fall out of the
  // same fold the codec uses. Variable fields (rpm, torque, alive)
  // are whatever the car sent; the constant bytes are checked too.
  const uint8_t loggedA8[8] = {0x39, 0x40, 0x01, 0x70, 0x01, 0x0C, 0xCF, 0x02};
  const uint8_t loggedA9[8] = {0x7D, 0x00, 0x97, 0xDF, 0x12, 0xC9, 0x7F, 0x01};
  const uint8_t loggedAA[8] = {0x52, 0x9E, 0x01, 0x00, 0xF8, 0x0D, 0x80, 0x81};
  const uint8_t logged2F3[3] = {0x0C, 0xFC, 0xFE};
  const uint8_t logged1D0[8] = {0x44, 0x3C, 0x66, 0xC6, 0x62, 0x93, 0x8D, 0xAC};
  expect(E92Checksum(0x0A8, loggedA8, 8, 0) == loggedA8[0], "logged 0x0A8 checksum");
  expect(refChecksum(0x0A8, loggedA8, 8, 0) == 0x39, "logged 0x0A8 ref");
  expect(loggedA8[6] == 0xCF, "logged 0x0A8 byte 6");
  expect(E92Checksum(0x0A9, loggedA9, 8, 0) == 0x7D, "logged 0x0A9 checksum");
  expect(E92Checksum(0x0AA, loggedAA, 8, 0) == 0x52, "logged 0x0AA checksum");
  expect(E92Checksum(0x2F3, logged2F3, 3, 2) == 0xFE, "logged 0x2F3 checksum");
  expect(logged1D0[1] == 0x3C && logged1D0[3] == 0xC6, "logged 0x1D0 constants");
  expect(E92AmbientHpa(logged1D0[3]) == 994, "logged ambient is 994 hPa");

  uint8_t m200[8];
  E92Build200(m200);
  const uint8_t want200[8] = {0x00, 0x80, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00};
  bool match200 = true;
  for (int i = 0; i < 8; i++)
    if (m200[i] != want200[i])
      match200 = false;
  expect(match200, "0x200 is 00 80 C0 00 00 00 00 00");

  uint8_t m1b6[7];
  E92Build1B6(m1b6, true);
  const uint8_t run1b6[7] = {0xFF, 0x35, 0x00, 0x00, 0x26, 0xA0, 0x15};
  const uint8_t off1b6[7] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0xAC, 0x00};
  bool matchRun = true;
  bool matchOff = true;
  for (int i = 0; i < 7; i++) {
    if (m1b6[i] != run1b6[i])
      matchRun = false;
  }
  E92Build1B6(m1b6, false);
  for (int i = 0; i < 7; i++) {
    if (m1b6[i] != off1b6[i])
      matchOff = false;
  }
  expect(matchRun, "0x1B6 running bytes");
  expect(matchOff, "0x1B6 off bytes");
  expect((run1b6[5] >> 4) == 0xA && (off1b6[5] >> 4) == 0xA, "0x1B6 nibble 0xA");
  expect(((off1b6[5] >> 2) & 0x3) == 3, "0x1B6 bits 3:2 are 3 when off");
  expect(((run1b6[5] >> 2) & 0x3) == 0, "0x1B6 bits 3:2 are 0 when running");

  uint8_t m3b3[6];
  E92Build3B3(m3b3, false, 0);
  const uint8_t off3b3[6] = {0x1F, 0x00, 0xFF, 0x7F, 0x00, 0xF1};
  const uint8_t run3b3[6] = {0x1F, 0xC8, 0x00, 0x00, 0x00, 0xF0};
  bool match3off = true;
  bool match3run = true;
  for (int i = 0; i < 6; i++)
    if (m3b3[i] != off3b3[i])
      match3off = false;
  E92Build3B3(m3b3, true, E92PowerByte(3000));
  for (int i = 0; i < 6; i++)
    if (m3b3[i] != run3b3[i])
      match3run = false;
  expect(match3off, "0x3B3 off / KL15 only");
  expect(match3run, "0x3B3 running at full power");
  expect(E92PowerByte(0) == 0, "power ramp starts at 0");
  expect(E92PowerByte(1500) == 0x64, "power ramp is halfway at 1.5 s");
  expect(E92PowerByte(3000) == 0xC8, "power ramp finishes at 0xC8");

  uint8_t nm[8];
  E92Build492(nm, E92NmWake);
  expect(nm[0] == 0x12 && nm[1] == 0x01, "NM wake is 12 01");
  E92Build492(nm, E92NmRing);
  expect(nm[0] == 0x17 && nm[1] == 0x02, "NM ring is 17 02");
  E92Build492(nm, E92NmStatus);
  expect(nm[0] == 0x17 && nm[1] == 0x42, "NM status is 17 42");
  E92Build492(nm, E92NmSteady);
  expect(nm[0] == 0x17 && nm[1] == 0x52, "NM steady is 17 52");
  bool nmTail = true;
  for (int i = 2; i < 8; i++)
    if (nm[i] != 0xFF)
      nmTail = false;
  expect(nmTail, "NM bytes 2-7 are FF");
  int phase = E92NmWake;
  int nStatus = 0;
  phase = E92NmNext(phase, &nStatus);
  expect(phase == E92NmRing, "wake is followed by the ring frame");
  phase = E92NmNext(phase, &nStatus);
  expect(phase == E92NmStatus, "ring is followed by 17 42");
  int statusCount = 0;
  while (phase == E92NmStatus && statusCount < 40) {
    phase = E92NmNext(phase, &nStatus);
    statusCount++;
  }
  expect(statusCount == E92NmStatusFrames, "17 42 lasts about 10 s");
  expect(phase == E92NmSteady, "then 17 52 for good");

  if (fails != 0) {
    std::printf("%d checks failed\n", fails);
    return 1;
  }
  std::printf("e92 msd80 host check ok\n");
  return 0;
}
