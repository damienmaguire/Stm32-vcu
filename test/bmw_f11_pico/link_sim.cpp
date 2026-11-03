// Private-link check: the ZV BMW_F11 class (F11Pico = 1) and the Pico
// v1z.1 (link_pico.c) on one pretend PT-CAN, car frames from scenario.h.
// Every frame one side sends, the other receives (the ZV only through
// the filters it registered). Nothing touches a real bus.
//
//   link_sim PHASE TRANS [o | p<ms> | l<ms>]
//     PHASE  ms offset of the ZV task grid, TRANS 1 automatic / 0 manual
//     on     link on, no fault (default)
//     o      link off (F11Pico = 0): the Pico mirrors the ZV from PT-CAN
//     p<ms>  the Pico goes silent from <ms>: the ZV must stop, show P,
//            set F11Fault 1 and not start again
//     l<ms>  the ZV goes silent from <ms>: the Pico must fall back
// Prints "<ms> C|Z|P id dlc bytes" per frame; checks and a summary on
// stderr, exit code 1 on a failed check.
#include "vehicles/BMW_F11.h"
#include <stdio.h>
#include <stdlib.h>
extern "C" {
#include "scenario.h"
}
#include "link_pico.h"
float Param::g[Param::PARAM_LAST];
static uint32_t T;
static int zvOff = -1, picoOff = -1;
static unsigned nFlt;
static uint32_t flt[16][2];
static unsigned picoN[5], picoBmw, zvPriv, zvMil, zvMilEarly;
static BMW_F11 car;
static void logFrame(char src, uint32_t id, const uint8_t *b, unsigned len) {
  printf("%u %c %03X %u", T, src, id, len);
  for (unsigned i = 0; i < len; i++)
    printf(" %02X", b[i]);
  printf("\n");
}
class LinkCan : public CanHardware {
public:
  void Send(uint32_t id, uint32_t data[2], uint8_t len, bool) override {
    const uint8_t *b = (const uint8_t *)data;
    if (zvOff >= 0 && (int)T >= zvOff)
      return;
    logFrame('Z', id, b, len);
    if (id >= 0x304 && id <= 0x307)
      zvPriv++;
    if (id == 0x592 && b[3] == 0x15) {
      zvMil++;
      if (picoOff < 0 || (int)T < picoOff)
        zvMilEarly++;
    }
    lp_rx((uint16_t)id, b, len, T * 1000u + 300u);
  }
  bool RegisterUserMessage(uint32_t id, uint32_t mask) override {
    flt[nFlt][0] = id;
    flt[nFlt][1] = mask;
    nFlt++;
    return true;
  }
};
static void zvRx(uint16_t id, const uint8_t *d) {
  bool pass = false;
  for (unsigned i = 0; i < nFlt; i++)
    pass |= (id & flt[i][1]) == (flt[i][0] & flt[i][1]);
  if (!pass)
    return;
  uint32_t w[2] = {0, 0};
  memcpy(w, d, 8);
  car.DecodeCAN(id, w);
}
static void carRx(uint16_t id, const uint8_t *d, uint8_t len) {
  uint8_t b[8] = {0};
  memcpy(b, d, len);
  logFrame('C', id, b, len);
  zvRx(id, b);
  lp_rx(id, b, len, T * 1000u);
}
static void picoTx(uint16_t id, const uint8_t d[8]) {
  logFrame('P', id, d, 8);
  if (id >= 0x308 && id <= 0x30C)
    picoN[id - 0x308]++;
  else
    picoBmw++;
  zvRx(id, d);
}
int main(int argc, char **argv) {
  int phase = argc > 1 ? atoi(argv[1]) : 3;
  int trans = argc > 2 ? atoi(argv[2]) : 1;
  char m = argc > 3 ? argv[3][0] : '-';
  bool linkOn = !(argc > 3 && !strcmp(argv[3], "o"));
  if (m == 'p')
    picoOff = atoi(argv[3] + 1);
  if (m == 'l')
    zvOff = atoi(argv[3] + 1);
  LinkCan can;
  Param::SetInt(Param::Transmission, trans);
  Param::SetInt(Param::F11Pico, linkOn);
  Param::SetFloat(Param::uaux, 13.8f);
  Param::SetFloat(Param::tmpm, 30.0f);
  car.SetCanInterface(&can);
  lp_init();
  int runAt = -1;
  unsigned privMs = 0, ptMs = 0, okMs = 0, fsMs = 0, frDiffMs = 0, starts = 0;
  unsigned fsAfterZv = 0, startsAfter = 0, fltBefore = 0, dirAfter = 0,
           lostAt = 0, stopAt = 0;
  bool sawD = false, sawR = false;
  const char *firstDiff = 0;
  uint32_t firstDiffT = 0;
  int lastEng = 0, lastLink = 0;
  for (T = 0; T < SCEN_END_MS; T++) {
    if (scen_bus(T) && !scen_bus(T - 1))
      lp_new_wake(); // FlexRay wakes with PT-CAN (not simulated here)
    scen_rx(T, carRx);
    int engp = Param::GetInt(Param::F11Eng);
    if (engp == 2 && runAt < 0)
      runAt = (int)T + 300;
    if (runAt >= 0 && (int)T >= runAt)
      Param::SetInt(Param::opmode, MOD_RUN);
    if (scen_stkl(T) < 0xA || !scen_bus(T)) {
      Param::SetInt(Param::opmode, 0);
      runAt = -1;
    }
    car.Task1Ms();
    if ((T + phase) % 10 == 0)
      car.Task10Ms();
    if ((T + phase) % 100 == 0)
      car.Task100Ms();
    if ((T + phase) % 200 == 0)
      car.Task200Ms();
    if ((T + phase) % 100 == 0) { // utils::SelectDirection, 100 ms task
      Vehicle::gear g;
      int dir;
      if (car.GetGear(g))
        dir = g == Vehicle::DRIVE ? 1 : (g == Vehicle::REVERSE ? -1 : 0);
      else
        dir = Param::GetInt(Param::opmode) == MOD_RUN ? 1 : 0;
      Param::SetInt(Param::dir, dir);
      sawD |= dir == 1;
      sawR |= dir == -1;
      if (lostAt && T >= lostAt + 100 && dir)
        dirAfter++;
    }
    bool picoUp = picoOff < 0 || (int)T < picoOff;
    lp_tick(T * 1000u + 500u, picoUp ? picoTx : 0);
    lp_state_t ps;
    lp_get(&ps);
    privMs += ps.src == 2;
    ptMs += ps.src == 1;
    // fail-safe while the car is awake (bus sleep ends the ZV's frames too)
    if ((T >= 1500 && T < 12000) || (T >= 14500 && T < 22000))
      fsMs += ps.failsafe;
    if (zvOff >= 0 && (int)T >= zvOff && ps.failsafe && ps.mode == 4)
      fsAfterZv++;
    int link = Param::GetInt(Param::F11Link);
    okMs += link == 2;
    if (link == 3 && lastLink != 3 && !lostAt)
      lostAt = T;
    lastLink = link;
    engp = Param::GetInt(Param::F11Eng);
    if (engp == 1 && lastEng != 1) {
      starts++;
      if (lostAt)
        startsAfter++;
    }
    if (lostAt && !stopAt && engp >= 3)
      stopAt = T;
    lastEng = engp;
    if (picoUp && Param::GetInt(Param::F11Fault) == 1)
      fltBefore++;
    const char *d = linkOn && picoUp && zvOff < 0 ? lp_fr_diff() : 0;
    if (d) {
      frDiffMs++;
      if (!firstDiff) {
        firstDiff = d;
        firstDiffT = T;
      }
    }
  }
  lp_state_t ps;
  lp_get(&ps);
  int fail = 0;
#define CHECK(c, ...)                                                          \
  if (!(c)) {                                                                  \
    fprintf(stderr, "FAIL: " __VA_ARGS__);                                     \
    fprintf(stderr, "\n");                                                     \
    fail = 1;                                                                  \
  }
  fprintf(stderr,
          "link %s phase %d trans %d: Pico 308 %u 309 %u 30A %u 30B %u 30C "
          "%u, Pico BMW ids %u, ZV 304-307 %u, Pico source private %u ms "
          "PT-CAN %u ms, link ok %u ms, ZV dropped %d, Pico dropped %u, "
          "starts %u, FlexRay private/PT-CAN differ %u ms",
          argc > 3 ? argv[3] : "on", phase, trans, picoN[0], picoN[1], picoN[2],
          picoN[3], picoN[4], picoBmw, zvPriv, privMs, ptMs, okMs,
          Param::GetInt(Param::F11LinkErr), (unsigned)ps.priv_bad, starts,
          frDiffMs);
  if (firstDiff)
    fprintf(stderr, " (first %s at %u ms)", firstDiff, firstDiffT);
  if (zvOff >= 0)
    fprintf(stderr, ", Pico fail-safe (MirrorFallback) %u ms after the ZV loss",
            fsAfterZv);
  if (picoOff >= 0)
    fprintf(stderr, ", lost at %u ms, engine stop at %u ms", lostAt, stopAt);
  fprintf(stderr, "\n");
  lp_fr_diff_print(stderr);
  CHECK(picoBmw == 0, "the Pico sent a BMW id");
  CHECK(picoN[0] > 100, "too few 0x308");
  CHECK(Param::GetInt(Param::F11LinkErr) == 0 && ps.priv_bad == 0,
        "dropped link frames");
  CHECK(fltBefore == 0, "F11Fault 1 while the Pico was up");
  CHECK(zvMilEarly == 0, "MIL on while the Pico was up");
  CHECK(!ps.fault_foreign, "Pico duplicate-ID fault");
  if (!linkOn) {
    CHECK(zvPriv == 0, "ZV sent 0x304-0x307 with the link off");
    CHECK(ptMs > 10000, "Pico did not mirror the ZV from PT-CAN");
  } else if (zvOff < 0) {
    CHECK(privMs > 10000 && ptMs < 100, "Pico not on the private source");
  }
  if (zvOff < 0 && picoOff < 0) {
    CHECK(starts == 2, "expected two starts, saw %u", starts);
    CHECK(fsMs == 0, "Pico fail-safe");
    if (trans)
      CHECK(sawD && sawR, "P-R-D not seen");
    if (linkOn)
      CHECK(lp_fr_check() == 0, "FlexRay from private differs from PT-CAN");
  }
  if (picoOff >= 0) {
    CHECK(lostAt && lostAt <= (unsigned)picoOff + 101,
          "ZV saw the loss at %u ms", lostAt);
    CHECK(stopAt && stopAt <= lostAt + 1, "engine not stopped");
    CHECK(dirAfter == 0, "direction after the loss");
    CHECK(startsAfter == 0, "restart while the Pico is lost");
    CHECK(zvMil > 0, "no MIL on 0x592");
    CHECK(Param::GetInt(Param::F11Fault) == 1, "F11Fault not 1");
  }
  if (zvOff >= 0)
    CHECK(ps.fallbacks > 0 && fsAfterZv > 1000,
          "no Pico fallback after ZV loss");
  fprintf(stderr, "%s\n", fail ? "FAILED" : "ok");
  return fail;
}
