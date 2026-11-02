// Pico v1z.1 private-link side for link_sim.cpp (see link_pico.c).
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef void (*lp_tx_fn)(uint16_t id, const uint8_t d[8]);
typedef struct {
  uint8_t src, mode;
  bool failsafe, fault_foreign;
  uint32_t fallbacks, priv_ok, priv_bad;
} lp_state_t;
void lp_init(void);
void lp_rx(uint16_t id, const uint8_t *d, uint8_t len, uint32_t us);
unsigned lp_tick(uint32_t us, lp_tx_fn tx);
void lp_new_wake(void); // FlexRay new wake (the Pico resets its mirror)
void lp_get(lp_state_t *s);
// FlexRay output fields of the main mirror and of a shadow mirror fed
// PT-CAN only; returns the name of the first differing field, or 0.
const char *lp_fr_diff(void);
void lp_fr_diff_print(FILE *f); // per field, ms differing
int lp_fr_check(void);          // discrete fields off for > 60 ms
#ifdef __cplusplus
}
#endif
