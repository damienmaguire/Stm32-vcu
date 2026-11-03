#!/bin/sh
# Host check: the ZV BMW_F11 class against the Pico v1h.3 PT-CAN core
# (pico-flexray src/ptcan_emu_core.c + dde_engine.c + dde_gear.c), both
# fed the same received frames (scenario.h). Nothing touches a bus.
#
#   PICO=/path/to/pico-flexray ./run.sh
#
# Checks: per id, the same payload states in the same order (CRC and
# alive counter left out), change times within one task period, moving
# values (rpm, torque, fuel, 0x0F3) within the Pico curve, and every
# ZV E2E CRC valid. Run for all ten 1 ms phases of the ZV task grid.
# Automatic: lever 0x197 and brake shifts (scenario.h) through
# dde_gear.c on the Pico side and the class's gear logic on the ZV side.
# 0x281 is left out on purpose: the ZV sends the real 12 V.
# Then the private link (F11Pico = 1) against the Pico v1z.1 zv_link.c
# (link_sim.cpp + link_pico.c): start, P-R-D, stop, Pico lost, ZV lost,
# link off; the Pico may send 0x308-0x30C only.
set -e
cd "$(dirname "$0")"
: "${PICO:?set PICO to the pico-flexray checkout}"
CXX=${CXX:-g++}
CC=${CC:-gcc}
OUT=${OUT:-/tmp/bmw_f11_pico}
mkdir -p "$OUT"
$CXX -std=c++14 -Wall -Wextra -Werror -Istub -I../../include -I. \
  -o "$OUT/zv_run" zv_run.cpp ../../src/vehicles/BMW_F11.cpp
$CC -std=c11 -Wall -Wextra -Werror -I"$PICO/tests/host/stubs" -I"$PICO/src" \
  -o "$OUT/pico_run" pico_run.c "$PICO/src/ptcan_emu_core.c" \
  "$PICO/src/dde_engine.c" "$PICO/src/dde_gear.c" -lm
"$OUT/pico_run" > "$OUT/pico.log" 2> "$OUT/pico.err"
fail=0
for ph in 0 1 2 3 4 5 6 7 8 9; do
  "$OUT/zv_run" $ph 1 > "$OUT/zv.log" 2> "$OUT/zv.err"
  "$OUT/pico_run" --crc "$OUT/zv.log" || fail=1
  python3 compare.py "$OUT/zv.log" "$OUT/pico.log" 281 > "$OUT/cmp$ph.txt" || true
  printf 'grid phase %s ms: ' $ph; tail -n 1 "$OUT/cmp$ph.txt"
  tail -n 1 "$OUT/cmp$ph.txt" | grep -q PASS || fail=1
done
# Manual car: the ZV must send exactly the Pico's DDE frames (no EGS ids,
# lever ignored). 0x3F9 is left out: its gear fields are the manual values
# on the ZV (fixed P on the Pico).
"$OUT/pico_run" m > "$OUT/pico_m.log" 2> "$OUT/pico_m.err"
for ph in 0 5; do
  "$OUT/zv_run" $ph 0 > "$OUT/zv_m.log" 2> "$OUT/zv_m.err"
  python3 compare.py "$OUT/zv_m.log" "$OUT/pico_m.log" 281,3F9 > "$OUT/cmpm$ph.txt" || true
  if grep -qE '^(3FD|39A|518|396) ' "$OUT/zv_m.log"; then echo "manual: EGS id sent"; fail=1; fi
  printf 'manual, grid phase %s ms: ' $ph; tail -n 1 "$OUT/cmpm$ph.txt"
  tail -n 1 "$OUT/cmpm$ph.txt" | grep -q PASS || fail=1
done
# Sleep scenario (no KL15, foreign NM 0x540 5-8 s): 0x512 pre frames,
# and silence once no NM keeps the bus awake (manual: from W+3.1 s).
for m in a m; do
  pa=s; tr=1; sk=281
  [ $m = m ] && { pa=ms; tr=0; sk=281,3F9; }
  "$OUT/pico_run" $pa > "$OUT/pico_s$m.log" 2> /dev/null
  for ph in 0 5; do
    "$OUT/zv_run" $ph $tr s > "$OUT/zv_s$m.log" 2> /dev/null
    python3 compare.py "$OUT/zv_s$m.log" "$OUT/pico_s$m.log" $sk > "$OUT/cmps$m$ph.txt" || true
    printf 'sleep %s, grid phase %s ms: ' $m $ph; tail -n 1 "$OUT/cmps$m$ph.txt"
    tail -n 1 "$OUT/cmps$m$ph.txt" | grep -q PASS || fail=1
  done
done
# Private link (needs the Pico v1z.1 or later: src/zv_link.c).
for f in link_pico.c "$PICO/src/zv_link.c" "$PICO/src/dde_engine.c" "$PICO/src/dde_gear.c"; do
  $CC -std=c11 -Wall -Wextra -Werror -DEMU_ZV_LINK=1 -I"$PICO/tests/host/stubs" \
    -I"$PICO/src" -c "$f" -o "$OUT/l_$(basename "$f" .c).o"
done
$CXX -std=c++14 -Wall -Wextra -Werror -Istub -I../../include -I. \
  -o "$OUT/link_sim" link_sim.cpp ../../src/vehicles/BMW_F11.cpp "$OUT"/l_*.o -lm
for tr in 1 0; do
  for ph in 0 5; do
    for m in on o p9000 p4300 l9000 l4300; do
      "$OUT/link_sim" $ph $tr $m > "$OUT/link_$tr$ph$m.log" 2> "$OUT/link_$tr$ph$m.txt" || fail=1
      head -n 1 "$OUT/link_$tr$ph$m.txt"; tail -n +2 "$OUT/link_$tr$ph$m.txt"
    done
  done
done
exit $fail
