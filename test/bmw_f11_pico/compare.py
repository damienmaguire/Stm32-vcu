import sys, bisect, collections
DYN = {'0A5':30,'0F3':30,'08F':40,'2C4':120}
E2E = {'08F','0A5','0D9','0DC','0F3','3F9','3FD','39A'}
TOL = {'0A5':10,'0D9':25,'0DC':20,'0F3':10,'08F':10,'2C4':100,'281':100,'1B9':200,'396':200,'3F9':10,'592':10,'3FD':5,'39A':5,'518':5,
       # 0x397: the Pico engine steps every 5 ms, and the random byte 2 events
       # (other draw times on the two boxes) can hold a change for the 15 ms gap
       '397':20}
def load(fn):
    d = collections.defaultdict(list)
    for l in open(fn):
        p = l.split()
        if len(p) < 3: continue
        t = int(p[0]); b = [int(x,16) for x in p[3:]]
        if p[1] in E2E: b = [0, b[1] & 0xF0] + b[2:]   # drop CRC and alive counter
        if p[1] == '3F9': b[4] = b[5] = 0   # real temperatures on the ZV (by design)
        # 0x397 byte 2 while running is random 0x5F..0x79 (new every 60 ms)
        if p[1] == '397' and 0x5F <= b[2] <= 0x79: b[2] = 0x5F
        d[p[1]].append((t, tuple(b)))
    return d
def changes(seq):
    out = []; last = None
    for t, b in seq:
        if b != last: out.append((t, b)); last = b
    return out
# numeric fields that move continuously: ZV value must lie between the Pico
# values seen in the window; every other byte must match one of them.
FIELDS = {'0A5': [lambda b: b[2] | (b[3] & 15) << 8, lambda b: b[3] >> 4 | b[4] << 4, lambda b: b[5] | b[6] << 8],
          '2C4': [lambda b: b[0] | b[1] << 8],
          '0F3': [lambda b: b[1] >> 4 | b[2] << 4]}
FBYTES = {'0A5': {2, 3, 4, 5, 6}, '2C4': {0, 1}, '0F3': {1, 2}}
def between(id_, b, cand):
    if id_ not in FIELDS or not cand: return False
    rest = lambda x: tuple(v for i, v in enumerate(x) if i not in FBYTES[id_])
    if not any(rest(c) == rest(b) for c in cand): return False
    for f in FIELDS[id_]:
        vs = [f(c) for c in cand]
        # slack: one Pico step (the ZV samples the same curve at another phase)
        # (at least 8 raw = the 2 rpm idle wobble of tri(t, 1200, 8))
        step = max([abs(x - y) for x, y in zip(vs, vs[1:])] + [8])
        if not (min(vs) - step <= f(b) <= max(vs) + step): return False
    return True
skip = set(sys.argv[3].split(',')) if len(sys.argv) > 3 else set()
z, p = load(sys.argv[1]), load(sys.argv[2])
fail = 0
for id_ in sorted(set(p) - set(z) - skip):
    print(f"{id_}: sent by the Pico only ({len(p[id_])} frames) -> DIFF"); fail += 1
for id_ in sorted(z):
    if id_ in skip: continue
    tol = TOL.get(id_, 10)
    zc, pc = changes(z[id_]), changes(p[id_])
    # payload states in order (consecutive duplicates removed)
    zs = [b for _, b in zc]; ps = [b for _, b in pc]
    worst = 0; mism = None
    if zs != ps:
        for i, (a, b) in enumerate(zip(zs, ps)):
            if a != b: mism = (i, zc[i], pc[i]); break
        if mism is None: mism = (min(len(zs), len(ps)), zc[len(ps):len(ps)+1], pc[len(zs):len(zs)+1])
    else:
        for (ta, _), (tb, _) in zip(zc, pc): worst = max(worst, abs(ta - tb))
    # per-wake first frame time
    def firsts(seq):
        f = []; prev = -10**9
        for t, _ in seq:
            if t - prev > 900: f.append(t)
            prev = t
        return f
    fz, fp = firsts(z[id_]), firsts(p[id_])
    st = 'OK'
    if id_ in DYN:
        # moving values (rpm, torque, fuel): each ZV frame must equal a Pico frame
        # within one Pico period +- the task grid offset.
        W = DYN[id_]; pt = [t for t, _ in p[id_]]; bad = 0; ex = None
        for t, b in z[id_]:
            i0 = bisect.bisect_left(pt, t - W); i1 = bisect.bisect_right(pt, t + W)
            cand = [p[id_][k][1] for k in range(i0, i1)]
            if not any(c == b for c in cand) and not between(id_, b, cand):
                bad += 1; ex = ex or (t, b, p[id_][i0:i1][:3])
        mism = None; worst = 0
        print(f"   window check +-{W} ms: {len(z[id_]) - bad}/{len(z[id_])} ZV frames match a Pico frame" + (f"; e.g. {ex}" if ex else ""))
        if bad: st = 'DIFF'; fail += 1
    elif mism or worst > tol: st = 'DIFF'; fail += 1
    print(f"{id_}: n zv {len(z[id_])} pico {len(p[id_])}; states zv {len(zs)} pico {len(ps)}; worst change-time diff {worst} ms (tol {tol}); first frames zv {fz} pico {fp} -> {st}")
    if mism:
        i, a, b = mism
        print("   first state mismatch #%d:\n     zv   %s\n     pico %s" % (i, a, b))
print("RESULT", "PASS" if not fail else f"{fail} DIFF")
