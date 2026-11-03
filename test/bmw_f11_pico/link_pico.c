// Pico side of the private-link check: the real Pico v1z.1 src/zv_link.c
// and dde_engine.c, wired as its own host co-sim (tests/host/zv_cosim/
// zv_cosim_pico.c): every PT-CAN frame goes to the duplicate-ID check
// and the mirror, every 1 ms the mirror steps and the due 0x308-0x30C
// frames are handed out. FlexRay is not simulated (0x309-0x30B say "no
// data"). A second, shadow mirror gets the same frames minus 0x304-0x307,
// so the FlexRay output built from the private link can be compared with
// the one the Pico would build from PT-CAN alone.
#include "link_pico.h"
#include "zv_link.h"
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static zvm_t zvm, sh;
static eng_t own, shOwn;
static zvt_t zvt;
static zvf_t zvf;
static zvl_status_t st;

void lp_init(void) {
  zvm_reset(&zvm);
  zvm_reset(&sh);
  memset(&own, 0, sizeof own);
  memset(&shOwn, 0, sizeof shOwn);
  eng_reset(&own);
  eng_reset(&shOwn);
  eng_set_mode(&own, ENG_MODE_CAS, 0);
  eng_set_mode(&shOwn, ENG_MODE_CAS, 0);
  zvf_reset(&zvf);
  zvt_reset(&zvt);
  zvt_set_on(&zvt, true, 0);
  memset(&st, 0, sizeof st);
}

void lp_rx(uint16_t id, const uint8_t *d, uint8_t len, uint32_t us) {
  zvt_on_rx(&zvt, id, us);
  if (zvm_wants_id(id)) {
    zvm_on_rx(&zvm, id, d, len, us);
    if (id < 0x304 || id > 0x307)
      zvm_on_rx(&sh, id, d, len, us);
  }
  if (id == 0x12Fu && len >= 7u) {
    eng_input_stkl(&own, d[2] & 0x0Fu, (d[6] & 3u) == 3u, us);
    eng_input_stkl(&shOwn, d[2] & 0x0Fu, (d[6] & 3u) == 3u, us);
  }
}

void lp_new_wake(void) {
  zvm_new_wake(&zvm);
  zvm_new_wake(&sh);
}

unsigned lp_tick(uint32_t us, lp_tx_fn tx) {
  zvm_pre_step(&zvm, &own);
  eng_step(&own, us);
  zvm_step(&zvm, &own, us);
  zvm_pre_step(&sh, &shOwn);
  eng_step(&shOwn, us);
  zvm_step(&sh, &shOwn, us);
  st.pico_mode = zvm_pico_mode(&zvm);
  st.src = zvm.src;
  st.zv_seen = st.mirror = zvm.src != ZVL_SRC_OWN;
  st.fallback = zvm.failsafe;
  st.mirror_us = zvm.in_us;
  unsigned n = 0;
  uint16_t id;
  uint8_t d[8];
  while (tx && zvt_next(&zvt, &zvf, &st, us, &id, d)) {
    tx(id, d);
    zvt_commit(&zvt, id, us);
    n++;
  }
  return n;
}

void lp_get(lp_state_t *s) {
  s->src = zvm.src;
  s->mode = zvm_pico_mode(&zvm);
  s->failsafe = zvm.failsafe;
  s->fault_foreign = zvt.fault_foreign;
  s->fallbacks = zvm.fallbacks;
  s->priv_ok = zvm.priv_ok;
  s->priv_bad = zvm.priv_bad_crc + zvm.priv_repeat + zvm.priv_bad_len;
}

// The fields FlexRay gets from the mirror (the PT-CAN-only gear bytes
// 0x3FD / 0x39A and g_park are left out; FlexRay uses g_fr_park).
#define F(x) {#x, offsetof(eng_out_t, x), sizeof(((eng_out_t *)0)->x)}
static const struct {
  const char *n;
  size_t o, s;
} kF[] = {F(active),   F(phase),   F(cnt_hi),  F(rpm4),     F(torque),
          F(turbine),  F(e6_b3),   F(e6_b4),   F(e6_ready), F(c3_b56),
          F(c3_b34),   F(r29_b5),  F(r29_b8),  F(f97c),     F(b142_b8),
          F(b142_b23), F(fb9_set), F(fb9_b2),  F(fb9_b3),   F(fb9_b4),
          F(fb7_mv),   F(e89),     F(e89_b1),  F(e8_fast),  F(e8_crank),
          F(g_set),    F(g_nib),   F(g_dc_b2), F(g_dc_b5),  F(g_fr_park)};

static unsigned nDiff[sizeof kF / sizeof kF[0]], run[sizeof kF / sizeof kF[0]],
    maxRun[sizeof kF / sizeof kF[0]], maxAbs[sizeof kF / sizeof kF[0]];

const char *lp_fr_diff(void) {
  const char *first = 0;
  if (zvm.src != ZVL_SRC_PRIV || sh.src != ZVL_SRC_PTCAN) {
    memset(run, 0, sizeof run);
    return 0;
  }
  for (size_t i = 0; i < sizeof kF / sizeof kF[0]; i++) {
    bool same = !memcmp((const char *)&zvm.out + kF[i].o,
                        (const char *)&sh.out + kF[i].o, kF[i].s);
    if (!strcmp(kF[i].n, "e89_b1") && zvm.out.e89 != 1)
      same = true; // only used with the running content (e89 = 1)
    if (!same) {
      unsigned va = 0, vb = 0;
      memcpy(&va, (const char *)&zvm.out + kF[i].o, kF[i].s);
      memcpy(&vb, (const char *)&sh.out + kF[i].o, kF[i].s);
      unsigned d = va > vb ? va - vb : vb - va;
      if (d > maxAbs[i])
        maxAbs[i] = d;
    }
    run[i] = same ? 0 : run[i] + 1;
    if (run[i] > maxRun[i])
      maxRun[i] = run[i];
    if (same)
      continue;
    if (memcmp((const char *)&zvm.out + kF[i].o,
               (const char *)&sh.out + kF[i].o, kF[i].s)) {
      nDiff[i]++;
      if (getenv("LP_DEBUG") && (nDiff[i] < 4 || nDiff[i] % 500 == 0)) {
        unsigned va = 0, vb = 0;
        memcpy(&va, (const char *)&zvm.out + kF[i].o, kF[i].s);
        memcpy(&vb, (const char *)&sh.out + kF[i].o, kF[i].s);
        fprintf(stderr, "dbg %s priv %u pt %u (n %u) ph %u/%u\n", kF[i].n, va,
                vb, nDiff[i], zvm.out.phase, sh.out.phase);
      }
      if (!first)
        first = kF[i].n;
    }
  }
  return first;
}

void lp_fr_diff_print(FILE *f) {
  for (size_t i = 0; i < sizeof kF / sizeof kF[0]; i++)
    if (nDiff[i])
      fprintf(f,
              "  FlexRay field %s differs %u ms, longest %u ms, by up to %u\n",
              kF[i].n, nDiff[i], maxRun[i], maxAbs[i]);
}

// Discrete fields must follow PT-CAN within 60 ms (one 0x3F9 / 0x0A5
// period plus the Pico's crank-marker timeout). rpm4, torque, turbine
// and fb7_mv are curves the PT-CAN mirror rebuilds on its own clock, so
// they are only reported.
int lp_fr_check(void) {
  int bad = 0;
  for (size_t i = 0; i < sizeof kF / sizeof kF[0]; i++) {
    const char *n = kF[i].n;
    if (!strcmp(n, "rpm4") || !strcmp(n, "torque") || !strcmp(n, "turbine") ||
        !strcmp(n, "fb7_mv"))
      continue;
    bad += maxRun[i] > 60;
  }
  return bad;
}
