#!/bin/sh
# Host check: the ZV BMW_SP51 class against Ian's close_v7 playback (the
# frames that closed a real SP51 pack on the bench) and a few abort
# scenarios with a simple pack model. Nothing touches a bus.
#
#   SP51_CSV=/path/to/sp51_close_v7_continuous.csv ./run.sh
set -e
cd "$(dirname "$0")"
: "${SP51_CSV:?set SP51_CSV to playback/sp51_close_v7_continuous.csv}"
CXX=${CXX:-g++}
OUT=${OUT:-/tmp/bmw_sp51}
mkdir -p "$OUT"
$CXX -std=c++14 -Wall -Wextra -Werror -Istub -I../../include -I. \
  -o "$OUT/sp51_run" sp51_run.cpp ../../src/bms/bmw_sp51.cpp
fail=0
"$OUT/sp51_run" v7 > "$OUT/v7.log" 2> "$OUT/v7.err" || fail=1
grep -E '^(ok|FAIL)' "$OUT/v7.err" | sed 's/^/v7: /'
python3 compare.py "$OUT/v7.log" "$SP51_CSV" > "$OUT/cmp.txt" || fail=1
cat "$OUT/cmp.txt"
for sc in cold otherbms notready holdlost holdpchk holdoff slowpch closetmo pchblock settle5 keyoff trip lost openreq fastopen sentinel late nopack suppoff ptcan; do
  "$OUT/sp51_run" $sc > "$OUT/$sc.log" 2> "$OUT/$sc.err" || fail=1
  grep -E '^(ok|FAIL)' "$OUT/$sc.err" | sed "s/^/$sc: /"
done
[ $fail = 0 ] && echo "ALL PASS" || echo "FAILED"
exit $fail
