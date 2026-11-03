#!/usr/bin/env python3
"""Compare the frames sp51_run sent with an SP51 playback CSV (SavvyCAN
GVRET, us timestamps). Per id: same count, every payload byte for byte
(CRC and alive counter included) and the same time within a tolerance.
0x6F1 (Ian's UDS reads) is not part of the ZV and is left out."""
import csv
import sys

# Allowed time difference in ms. The 1 s frames are spread over the odd
# 10 ms ticks on the ZV, the rest go out on the same tick as the playback.
TOL = {0x328: 20, 0x3A0: 40, 0x3CA: 60, 0x2CA: 80, 0x3E8: 100, 0x510: 120,
       0x512: 140, 0x330: 160}
SKIP = {0x6F1}


def load_ref(path):
    out = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            i = int(row["ID"], 16)
            if i in SKIP:
                continue
            n = int(row["LEN"])
            data = tuple(int(row["D%d" % k], 16) for k in range(1, n + 1))
            out.setdefault(i, []).append((int(row["Time Stamp"]) / 1000.0, data))
    return out


def load_zv(path):
    out = {}
    with open(path) as f:
        for line in f:
            p = line.split()
            i = int(p[1], 16)
            data = tuple(int(x, 16) for x in p[3:3 + int(p[2])])
            out.setdefault(i, []).append((float(p[0]), data))
    return out


def main():
    zv, ref = load_zv(sys.argv[1]), load_ref(sys.argv[2])
    bad = 0
    for i in sorted(set(zv) | set(ref)):
        a, b = zv.get(i, []), ref.get(i, [])
        tol = TOL.get(i, 1)
        msg = []
        if len(a) != len(b):
            msg.append("count zv %d ref %d" % (len(a), len(b)))
        for k, ((ta, da), (tb, db)) in enumerate(zip(a, b)):
            if da != db:
                msg.append("frame %d at %.3f s: zv %s ref %s" % (
                    k, tb / 1000, " ".join("%02X" % x for x in da),
                    " ".join("%02X" % x for x in db)))
                break
            if abs(ta - tb) > tol:
                msg.append("frame %d time zv %.3f ref %.3f" % (k, ta, tb))
                break
        print("%03X %5d frames  %s" % (i, len(b), "; ".join(msg) or "same"))
        bad += bool(msg)
    print("PASS" if not bad else "FAIL (%d ids differ)" % bad)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
