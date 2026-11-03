// Host check of the BMW_SP51 class against a simple model of the pack.
// Prints every frame the class sends as "t_ms id dlc bytes" on stdout,
// and the checks of the chosen scenario on stderr. Nothing touches a bus.
//
//   sp51_run v7        the close_v7 timeline (compare.py against the CSV)
//   sp51_run early     key on 2 s after power up, before the pack settled:
//                      the ZV precharge times out, the pack is never asked
//                      to close, and the next key on works
//   sp51_run keyoff    key off in Run: the pack stays closed for the ZV's
//                      2.5 s shutdown delay, then opens
//   sp51_run trip      the ZV drops its contactor outputs with no delay
//                      (as its over voltage trip does when stationary)
//   sp51_run lost      0x112 stops while closed
//   sp51_run openreq   the pack asks to open while closed
//   sp51_run nopack    the pack never answers (one 1 s wake retry)
//   sp51_run suppoff   SP51Supp off: only 0x10B and 0x53A
//   sp51_run ptcan     battery on the F11 PT-CAN: only 0x10B and 0x53A
#include "bms/bmw_sp51.h"
#include "digio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

float Param::g[Param::PARAM_LAST];
DigIo DigIo::prec_out;
DigIo DigIo::dcsw_out;

static uint32_t T;          // ms
static uint32_t lastTx = 0; // ms of the last frame we sent
static uint32_t maxGap = 0; // longest silence once the pack answered
static int packCmd = 0;     // last 0x10B command the pack saw
static int ids[0x800];      // frames sent per id
static int firstClose = -1; // ms of the first 0x10B CLOSE
static int lastClose = -1;  // ms of the last 0x10B CLOSE
static bool packAwake = false;

class LogCan : public CanHardware {
public:
  void Send(uint32_t id, uint32_t data[2], uint8_t len, bool) override {
    const uint8_t *b = (const uint8_t *)data;
    printf("%u %03X %u", T, id, len);
    for (int i = 0; i < len; i++)
      printf(" %02X", b[i]);
    printf("\n");
    if (packAwake && T - lastTx > maxGap)
      maxGap = T - lastTx;
    lastTx = T;
    ids[id & 0x7FF]++;
    if (id == 0x10B) {
      packCmd = b[1] >> 4;
      if (packCmd == 1) {
        if (firstClose < 0)
          firstClose = T;
        lastClose = T;
      }
    }
  }
  bool RegisterUserMessage(uint32_t, uint32_t) override { return true; }
};

static BMW_SP51 bms;
static int fails = 0;

static void check(bool ok, const char *what) {
  fprintf(stderr, "%s: %s\n", ok ? "ok  " : "FAIL", what);
  if (!ok)
    fails++;
}

static void rx(int id, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3,
               uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7) {
  uint8_t d[8] = {b0, b1, b2, b3, b4, b5, b6, b7};
  bms.DecodeCAN(id, d);
}

int main(int argc, char **argv) {
  const char *sc = argc > 1 ? argv[1] : "v7";
  bool v7 = !strcmp(sc, "v7");
  LogCan can;
  Param::SetInt(Param::SP51Supp, strcmp(sc, "suppoff") ? 1 : 0);
  Param::SetInt(Param::BMSCan, 1);
  Param::SetInt(Param::VehicleCan, !strcmp(sc, "ptcan") ? 1 : 0);
  Param::SetInt(Param::Vehicle, vBMW_F11);
  Param::SetInt(Param::ShuntType, 0);
  Param::SetInt(Param::opmode, MOD_OFF);
  bms.SetCanInterface(&can);

  // Pack model: answers 100 ms after our first frame, contactor state 4
  // (open), 5 (precharge, 80 ms after CLOSE), 6 (closed, 1.86 s after
  // CLOSE), back to 4 70 ms after OPEN. 0x1FA once a second plus an
  // extra frame on every change, as the real pack does.
  int hv = 4, target = 4, changeAt = -1, seenCmd = 0;
  int answers = strcmp(sc, "nopack") != 0;
  // ZV model: the opmode and the prec_out / dcsw_out handling of
  // Ms10Task in stm32_vcu.cpp, rlyDly delays included.
  bool early = !strcmp(sc, "early");
  bool trip = !strcmp(sc, "trip");
  int keyOn = v7 ? 24900 : (early ? 2000 : 10000);
  // v7: the playback opens at 40.0 s, so the key goes off 2.5 s before.
  int keyOff = v7 ? 37490 : 45000;
  int minTime = 0, pchStart = -1, runAt = -1, rlyDly = 0;
  int pchFailAt = -1, dcswOffAt = -1;
  const int end = v7 ? 50000 : 60000;

  for (T = 0; T < (uint32_t)end; T += 10) {
    // --- the pack ---
    if (answers && T >= 100)
      packAwake = true;
    bool stop112 = !strcmp(sc, "lost") && T >= 32000;
    int openReq = (!strcmp(sc, "openreq") && T >= 32000) ? 2 : 1;
    if (packAwake) {
      if (packCmd != seenCmd) {
        seenCmd = packCmd;
        target = packCmd ? 5 : 4;
        changeAt = T + (packCmd ? 80 : 70);
      }
      int prev = hv;
      if (changeAt >= 0 && (int)T >= changeAt) {
        hv = target;
        if (hv == 5) {
          target = 6;
          changeAt = T + 1780;
        } else
          changeAt = -1;
      }
      uint8_t b1 = (hv << 4) | 0x05;
      if (hv != prev || T % 1000 == 500)
        rx(0x1FA, 0, b1, 0, 0, 0, 0, 72, 74); // cells 22 / 24 C
      if (T % 20 == 10 && !stop112) {
        uint16_t i = 8192, u = 3808; // 0.0 A, 380.8 V
        rx(0x112, i, i >> 8, u, u >> 8, 0, openReq << 6, 0,
           hv == 6 ? 0x5F : 0x00);
      }
      if (T % 100 == 30)
        rx(0x2F5, 0x04, 0x10, 0xE4, 0x47, 0x54, 0x0B, 0xB8, 0x0B);
      if (T % 100 == 50)
        rx(0x40D, 0xE5, 0x3B, 0x20, 0x4E, 0x80, 0x3E, 0x08, 0x52);
      if (T % 200 == 70)
        rx(0x432, 0, 0, 0, 0, 0xFF, 0, 0, 0);
    }

    // --- the ZV: Ms10Task runs the BMS first, then the mode control ---
    bms.Task10Ms();
    int op = Param::GetInt(Param::opmode);
    switch (op) {
    case MOD_OFF:
      if (rlyDly)
        rlyDly--; // shutdown delay before opening the contactors
      if (!rlyDly && (DigIo::dcsw_out.Get() || DigIo::prec_out.Get())) {
        DigIo::dcsw_out.Clear();
        DigIo::prec_out.Clear();
        dcswOffAt = T;
      }
      if ((int)T == keyOn) {
        op = MOD_PRECHARGE;
        rlyDly = 25;
        minTime = 100;
        pchStart = T;
      }
      break;
    case MOD_PRECHARGE:
      if (rlyDly)
        rlyDly--;
      if (!rlyDly)
        DigIo::prec_out.Set();
      if (minTime)
        minTime--;
      if (!minTime &&
          Param::GetFloat(Param::udc) >= Param::GetFloat(Param::udcsw)) {
        op = MOD_RUN;
        rlyDly = 25;
        runAt = T;
      } else if ((int)T - pchStart > 5000) {
        DigIo::prec_out.Clear();
        op = MOD_PCHFAIL;
        pchFailAt = T;
      }
      break;
    case MOD_PCHFAIL:
      DigIo::prec_out.Clear();
      if (early && (int)T == 10000) {
        op = MOD_OFF; // key off, then on again at 12 s
        keyOn = 12000;
      }
      break;
    case MOD_RUN:
      if (rlyDly)
        rlyDly--;
      if (!rlyDly)
        DigIo::dcsw_out.Set();
      if (trip && T == 32000) {
        DigIo::dcsw_out.Clear(); // outputs off, no delay
        DigIo::prec_out.Clear();
        op = MOD_OFF;
        dcswOffAt = T;
      } else if ((int)T == keyOff) {
        op = MOD_OFF;
        rlyDly = 250; // the ZV's 2.5 s delayed shutdown
      }
      break;
    }
    Param::SetInt(Param::opmode, op);

    if (T % 100 == 0)
      bms.Task100Ms();
    if (T % 500 == 0)
      fprintf(stderr, "%6u op %d state %d fault %d hv %d udc %.1f udcsw %.1f\n",
              T, op, Param::GetInt(Param::SP51State),
              Param::GetInt(Param::SP51Fault), Param::GetInt(Param::SP51HvSt),
              Param::GetFloat(Param::udc), Param::GetFloat(Param::udcsw));
  }

  // --- checks per scenario ---
  int fault = Param::GetInt(Param::SP51Fault);
  check(!answers || maxGap <= 20,
        "no TX gap over 20 ms once the pack answered");
  check(ids[0x380] == 0, "0x380 never sent");
  if (v7) {
    check(firstClose == 27000 && lastClose == 39980,
          "CLOSE from 27.0 s to 39.98 s");
    check(runAt > 0 && runAt - pchStart <= 5000,
          "ZV precharge done within 5 s");
    fprintf(stderr, "      precharge took %d ms, run at %d ms\n",
            runAt - pchStart, runAt);
    check(Param::GetFloat(Param::BMS_ChargeLim) > 120 &&
              Param::GetFloat(Param::BMS_ChargeLim) < 121,
          "charge limit capped by power (46 kW / 380.8 V)");
    check(Param::GetFloat(Param::BMS_MaxOutput) > 59.9f,
          "discharge power 60 kW");
    check(Param::GetInt(Param::SP51State) == BMW_SP51::READY, "ready again");
    check(Param::GetFloat(Param::udc2) > 380.7f &&
              Param::GetFloat(Param::udc) == 0,
          "open: udc2 = pack side 380.8 V, udc = link side 0 V");
  } else if (!strcmp(sc, "early")) {
    // 0x112 starts at 0.11 s and is settled 5 s later; with 2 s of B the
    // close would be due after the ZV's 5 s precharge timeout.
    check(pchFailAt >= 7000 && pchFailAt < 7100, "ZV precharge fails at 7 s");
    check(firstClose >= 12000, "no CLOSE before the second key on");
    check(runAt > 12000 && runAt - pchStart <= 5000,
          "second key on: precharge done within 5 s");
    fprintf(stderr, "      first CLOSE %d ms, run at %d ms\n", firstClose,
            runAt);
  } else if (!strcmp(sc, "keyoff")) {
    check(dcswOffAt == keyOff + 2500, "ZV outputs off 2.5 s after key off");
    check(lastClose >= keyOff + 2490 && lastClose <= keyOff + 2520,
          "pack held closed for the 2.5 s delay, then OPEN");
    check(fault == BMW_SP51::NONE, "no fault");
    check(Param::GetInt(Param::SP51State) == BMW_SP51::READY, "ready again");
    fprintf(stderr,
            "      key off %d ms, outputs off %d ms, last CLOSE %d ms\n",
            keyOff, dcswOffAt, lastClose);
  } else if (trip) {
    check(lastClose >= 31990 && lastClose <= 32020,
          "OPEN on the next 0x10B after the outputs drop");
    check(fault == BMW_SP51::NONE, "no fault");
  } else if (!strcmp(sc, "lost")) {
    check(lastClose >= 32400 && lastClose <= 32540,
          "OPEN within 0.55 s of 0x112 stopping");
    check(fault == BMW_SP51::PACK_LOST, "fault PackLost");
    check(Param::GetFloat(Param::udcsw) == 500, "udcsw held at 500");
  } else if (!strcmp(sc, "openreq")) {
    check(lastClose >= 31990 && lastClose <= 32010, "OPEN on the next 0x10B");
    check(fault == BMW_SP51::OPEN_REQ, "fault OpenReq");
  } else if (!strcmp(sc, "nopack")) {
    check(firstClose < 0, "never CLOSE");
    check(ids[0x10B] == (end - 1000) / 20, "one 1 s wake silence only");
  } else if (!strcmp(sc, "suppoff") || !strcmp(sc, "ptcan")) {
    int others = 0;
    for (int i = 0; i < 0x800; i++)
      if (i != 0x10B && i != 0x53A)
        others += ids[i];
    check(others == 0 && ids[0x10B] && ids[0x53A], "only 0x10B and 0x53A");
    check(firstClose > 0, "still closes");
  }
  return fails ? 1 : 0;
}
