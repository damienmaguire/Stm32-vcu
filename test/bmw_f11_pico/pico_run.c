// Runs the Pico v1h.3 PT-CAN core + engine + gear on the shared scenario.
// pico_run m = manual car: DDE frames only, gear off (PTE_MASK_DDE).
// An 's' in the argument (s, ms) = the sleep scenario (scenario.h).
// Same output format as zv_run. Also: pico_run --crc <file> checks E2E of a log.
#include "ptcan_emu_core.h"
#include "dde_engine.h"
#include "dde_gear.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "scenario.h"
static pte_t P;
static eng_t E;
static gear_t G;
static int Auto = 1;
static uint32_t Tms;
static uint32_t lastRx = 0;
static void rx(uint16_t id, const uint8_t *d, uint8_t len) {
  uint64_t us = (uint64_t)Tms * 1000u + 1000000u;
  // In the car FlexRay wakes with PT-CAN; the Pico resets its engine on a FlexRay wake.
  if (Tms - lastRx >= 1000u) { eng_reset(&E); gear_reset(&G, Auto); }
  lastRx = Tms;
  pte_on_rx(&P, id, d, len, us);
  if (id == 0x12F) eng_input_stkl(&E, d[2] & 0xF, (d[6] & 3) == 3, (uint32_t)us);
  if (id == 0x173 && (d[7] & 3) == 3) { eng_input_brake(&E, true, (uint32_t)us); gear_input_brake(&G, (uint32_t)us); }
  if (id == 0x197) gear_input_lever(&G, d, len, (uint32_t)us);
}
static int crc_check(const char *fn) {
  FILE *f = fopen(fn, "r");
  unsigned t, id, dlc, b[8]; int bad = 0, ok = 0;
  char line[256];
  while (fgets(line, sizeof line, f)) {
    if (sscanf(line, "%u %x %u %x %x %x %x %x %x %x %x", &t, &id, &dlc, &b[0], &b[1], &b[2], &b[3], &b[4], &b[5], &b[6], &b[7]) < 3) continue;
    int i = pte_index_of((uint16_t)id);
    if (i < 0 || pte_defs[i].data_id == 0xFFFF) continue;
    uint8_t d[8]; for (unsigned k = 0; k < dlc; k++) d[k] = (uint8_t)b[k];
    if (d[0] == pte_e2e_crc(pte_defs[i].data_id, d, (uint8_t)dlc)) ok++; else { bad++; if (bad < 5) printf("bad crc %u %03X\n", t, id); }
  }
  printf("E2E: %d ok, %d bad\n", ok, bad);
  return bad != 0;
}
int main(int argc, char **argv) {
  if (argc > 2) return crc_check(argv[2]);
  Auto = !(argc > 1 && strchr(argv[1], 'm'));
  scen_sleep = argc > 1 && strchr(argv[1], 's') != NULL;
  pte_init(&P);
  pte_set_mask(&P, Auto ? PTE_MASK_ALL : PTE_MASK_DDE);
  eng_reset(&E);
  gear_reset(&G, Auto);
  eng_set_profile(&E, ENG_PROF_SHORT);
  for (Tms = 0; Tms < SCEN_END_MS; Tms++) {
    scen_rx(Tms, rx);
    for (unsigned sub = 0; sub < 10; sub++) {
      uint64_t us = (uint64_t)Tms * 1000u + 1000000u + sub * 100u;
      if (us % 5000u == 0) {
        if (us % 1000000u == 0) eng_set_mode(&E, ENG_MODE_CAS, (uint32_t)us);
        eng_step(&E, (uint32_t)us);
      }
      if (sub == 0) {   // core 1 gear_tick, every 1 ms
        gear_step(&G, (uint32_t)us, E.phase == ENG_RUN);
        gear_fill_out(&G, &E.out);
      }
      P.eng = E.out.active ? &E.out : NULL;
      pte_tx_t tx;
      while (pte_next(&P, us, &tx)) {
        printf("%u %03X %u", Tms, tx.id, tx.dlc);
        for (int i = 0; i < tx.dlc; i++) printf(" %02X", tx.data[i]);
        printf("\n");
        pte_commit(&P, &tx, us);
      }
    }
    if (Tms % 50 == 0) fprintf(stderr, "%u eng %u\n", Tms, E.phase);
  }
  return 0;
}
