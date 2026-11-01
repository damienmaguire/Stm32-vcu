// Runs the ZV BMW_F11 class on the shared scenario; prints "t id dlc bytes".
#include "vehicles/BMW_F11.h"
#include <stdio.h>
#include <stdlib.h>
extern "C" {
#include "scenario.h"
}
float Param::g[Param::PARAM_LAST];
static uint32_t T;
class LogCan : public CanHardware {
public:
  void Send(uint32_t id, uint32_t data[2], uint8_t len, bool) override {
    const uint8_t *b = (const uint8_t *)data;
    printf("%u %03X %u", T, id, len);
    for (int i = 0; i < len; i++) printf(" %02X", b[i]);
    printf("\n");
  }
  bool RegisterUserMessage(uint32_t, uint32_t) override { return true; }
};
static BMW_F11 car;
static void rx(uint16_t id, const uint8_t *d, uint8_t len) {
  (void)len;
  uint32_t w[2];
  memcpy(w, d, 8);
  car.DecodeCAN(id, w);
}
int main(int argc, char **argv) {
  int phase = argc > 1 ? atoi(argv[1]) : 3; // ms offset of the task grid
  int trans = argc > 2 ? atoi(argv[2]) : 1;
  int noRun = argc > 3 && argv[3][0] == 'n';    // VCU never reaches RUN
  int foreign = argc > 3 && argv[3][0] == 'f';  // a second box sends 0x3F9 at 6 s
  LogCan can;
  Param::SetInt(Param::Transmission, trans);
  Param::SetFloat(Param::uaux, 13.8f);
  Param::SetFloat(Param::tmpm, 30.0f);
  car.SetCanInterface(&can);
  int runAt = -1;
  for (T = 0; T < SCEN_END_MS; T++) {
    scen_rx(T, rx);
    // The VCU: RUN 300 ms after our story says running, off with KL15.
    int engp = Param::GetInt(Param::F11Eng);
    if (engp == 2 && runAt < 0) runAt = (int)T + 300;
    if (foreign && T == 6000) { uint8_t d[8] = {0}; rx(0x3F9, d, 8); }
    if (!noRun && runAt >= 0 && (int)T >= runAt) Param::SetInt(Param::opmode, MOD_RUN);
    if (scen_stkl(T) < 0xA || !scen_bus(T)) { Param::SetInt(Param::opmode, 0); runAt = -1; }
    car.Task1Ms();
    if ((T + phase) % 10 == 0) car.Task10Ms();
    if ((T + phase) % 100 == 0) car.Task100Ms();
    if ((T + phase) % 200 == 0) car.Task200Ms();
    // utils::SelectDirection asks GetGear every 200 ms.
    if ((T + phase) % 200 == 0) {
      Vehicle::gear g;
      if (car.GetGear(g))
        Param::SetInt(Param::dir, g == Vehicle::DRIVE ? 1 : (g == Vehicle::REVERSE ? -1 : 0));
      else
        Param::SetInt(Param::dir, Param::GetInt(Param::opmode) == MOD_RUN ? 1 : 0);
    }
    if (T % 50 == 0)
      fprintf(stderr, "%u eng %d fault %d dir %d opmode %d gear %d\n", T, Param::GetInt(Param::F11Eng),
              Param::GetInt(Param::F11Fault), Param::GetInt(Param::dir), Param::GetInt(Param::opmode),
              Param::GetInt(Param::F11Gear));
  }
  return 0;
}
