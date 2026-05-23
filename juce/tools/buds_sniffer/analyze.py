#!/usr/bin/env python3
"""Analyze protocol_log.txt from buds_sniffer to nail down quaternion conventions.

For each phase, take the middle ~3 s as the "settled" window and compute
the mean raw-quaternion. Then compute delta-quaternions relative to the
baseline rest pose, under both XYZW and WXYZ hypotheses, and report which
component of each delta is dominant — that tells us which slot is the
scalar w and which slots correspond to body-frame axes.
"""

import sys
import re
import math
from collections import defaultdict

LOG = "protocol_log.txt"

def parse(path):
    phases = defaultdict(list)  # phase_tag -> [(ms, q0,q1,q2,q3,valid)]
    phase_bounds = {}            # tag -> (begin_ms, end_ms)
    cur_begin = None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'): continue
            parts = line.split()
            if parts[0] == 'PHASE_BEGIN':
                cur_begin = (parts[1], int(parts[2]))
            elif parts[0] == 'PHASE_END':
                tag, t1 = parts[1], int(parts[2])
                if cur_begin and cur_begin[0] == tag:
                    phase_bounds[tag] = (cur_begin[1], t1)
            elif parts[0] == 'GRV':
                tag = parts[1]
                ms = int(parts[2])
                raws = list(map(int, parts[3:7]))
                phases[tag].append((ms, *raws))
    return phases, phase_bounds

def mean_in_middle(samples, bounds, margin_frac=0.25):
    """Mean over the middle (1-2*margin_frac) of the phase window."""
    if not samples or not bounds: return None
    t0, t1 = bounds
    dur = t1 - t0
    a = t0 + dur * margin_frac
    b = t1 - dur * margin_frac
    sel = [s for s in samples if a <= s[0] <= b]
    if not sel: sel = samples
    n = len(sel)
    q = [0.0]*4
    for s in sel:
        for i in range(4):
            q[i] += s[1 + i] / 10000.0
    q = [v / n for v in q]
    # Re-normalize (averaging breaks unit length slightly).
    mag = math.sqrt(sum(v*v for v in q))
    if mag > 0:
        q = [v / mag for v in q]
    return q, n

# Quaternion algebra (Hamilton convention, scalar-first wxyz).
def qmul(a, b):
    aw,ax,ay,az = a
    bw,bx,by,bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )

def qconj(q):
    w,x,y,z = q
    return (w, -x, -y, -z)

def to_axis_angle(q):
    w,x,y,z = q
    # canonicalize sign so w >= 0 → angle in [0, 180]
    if w < 0:
        w,x,y,z = -w,-x,-y,-z
    w = max(-1.0, min(1.0, w))
    angle = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w*w))
    if s < 1e-6:
        return (1.0, 0.0, 0.0), angle
    return (x/s, y/s, z/s), angle

def slot_label(raw_idx, hypothesis):
    # hypothesis 'wxyz': raw[0]=w, raw[1]=x, raw[2]=y, raw[3]=z
    # hypothesis 'xyzw': raw[0]=x, raw[1]=y, raw[2]=z, raw[3]=w
    if hypothesis == 'wxyz': return ['w','x','y','z'][raw_idx]
    return ['x','y','z','w'][raw_idx]

def to_wxyz(q_raw, hyp):
    """Re-order raw [q0,q1,q2,q3] into canonical (w,x,y,z) tuple under hypothesis."""
    if hyp == 'wxyz': return (q_raw[0], q_raw[1], q_raw[2], q_raw[3])
    else:             return (q_raw[3], q_raw[0], q_raw[1], q_raw[2])

def main():
    phases, bounds = parse(LOG)
    print(f"Parsed phases: {sorted(phases.keys())}")
    print()

    # Compute mean raw quaternion per phase.
    means = {}
    for tag, samples in phases.items():
        m = mean_in_middle(samples, bounds.get(tag))
        if m:
            means[tag] = m[0]
            print(f"  {tag:12s} n={m[1]:3d}  q_raw=[{m[0][0]:+0.4f} {m[0][1]:+0.4f} {m[0][2]:+0.4f} {m[0][3]:+0.4f}]")
    print()

    # Baseline = average of all rest phases.
    rest_keys = [k for k in means if k.startswith('rest_')]
    if not rest_keys:
        print("No rest phases found.")
        return
    baseline_raw = [0.0]*4
    for k in rest_keys:
        for i in range(4):
            baseline_raw[i] += means[k][i]
    baseline_raw = [v / len(rest_keys) for v in baseline_raw]
    mag = math.sqrt(sum(v*v for v in baseline_raw))
    baseline_raw = [v / mag for v in baseline_raw]
    print(f"baseline (mean of rests, raw slot order): [{baseline_raw[0]:+0.4f} {baseline_raw[1]:+0.4f} {baseline_raw[2]:+0.4f} {baseline_raw[3]:+0.4f}]")
    print()

    gestures = ['yaw_left','yaw_right','pitch_down','pitch_up','roll_left','roll_right']

    for hyp in ('xyzw','wxyz'):
        print(f"========== Hypothesis: wire order = {hyp} ==========")
        q_base = to_wxyz(baseline_raw, hyp)
        for g in gestures:
            if g not in means: continue
            q_g = to_wxyz(means[g], hyp)
            # delta = q_g * conj(q_base)  →  rotation taking baseline to gesture
            q_delta = qmul(q_g, qconj(q_base))
            axis, ang = to_axis_angle(q_delta)
            print(f"  {g:12s} Δ=[w={q_delta[0]:+0.4f} x={q_delta[1]:+0.4f} y={q_delta[2]:+0.4f} z={q_delta[3]:+0.4f}]  "
                  f"axis=({axis[0]:+0.3f},{axis[1]:+0.3f},{axis[2]:+0.3f}) ang={math.degrees(ang):+6.1f}°")
        print()

    # Heuristic: under the correct hypothesis the SAME gesture-family (yaw, pitch, roll)
    # should produce deltas whose rotation axes are opposite for left/right pairs.
    print("Sanity check: for each pair, axes should be roughly antiparallel under the right hypothesis.")
    for hyp in ('xyzw','wxyz'):
        print(f"  --- {hyp} ---")
        q_base = to_wxyz(baseline_raw, hyp)
        for a,b in [('yaw_left','yaw_right'), ('pitch_down','pitch_up'), ('roll_left','roll_right')]:
            if a not in means or b not in means: continue
            qa = qmul(to_wxyz(means[a], hyp), qconj(q_base))
            qb = qmul(to_wxyz(means[b], hyp), qconj(q_base))
            axa, _ = to_axis_angle(qa); axb, _ = to_axis_angle(qb)
            dot = axa[0]*axb[0] + axa[1]*axb[1] + axa[2]*axb[2]
            print(f"    {a:12s} vs {b:12s}  axis·axis = {dot:+0.3f}  (closer to -1 = correct hypothesis)")

if __name__ == '__main__':
    main()
