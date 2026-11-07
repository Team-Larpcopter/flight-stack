#!/usr/bin/env python3
"""Compare two VIO trajectories (CSV from stationary_test.py) after alignment.

    compare_vio.py A.csv B.csv [--names sqrtVINS cuVSLAM] [--out compare.json]

The two estimators use different world frames (sqrtVINS: gravity-aligned
"global" with arbitrary yaw; cuVSLAM: "odom" anchored at the start pose), so
positions cannot be subtracted directly. This tool:

  1. time-aligns: B is linearly interpolated onto A's timestamps (overlap only)
  2. expresses both relative to their first sample
  3. finds the rigid rotation R + translation t that best maps B onto A
     (Umeyama / Horn, least squares, no scale: both are metric)
  4. reports the disagreement between the aligned tracks (RMS, max, final),
     each track's own drift from its start (end and max), path lengths, and
     the rotation needed to align them (heading offset between the frames)

For a still drone, both tracks should stay near 0 and the "between" numbers
mostly show each estimator's own drift and jitter. For hand-held motion the
between-RMS says how closely they agree on the same motion.
"""

import argparse
import csv
import json
import math

import numpy as np


def load(path):
    rows = list(csv.DictReader(open(path)))
    # receive time (shared clock); sqrtVINS header stamps are float32-quantised
    key = "recv_s" if rows and "recv_s" in rows[0] else "stamp_s"
    t = np.array([float(r[key]) for r in rows])
    p = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
    order = np.argsort(t, kind="stable")
    return t[order], p[order]


def umeyama(src, dst):
    """R, t minimising ||dst - (R src + t)||^2 (no scale)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    cov = (dst - mu_d).T @ (src - mu_s) / len(src)
    U, _, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    return R, mu_d - R @ mu_s


def path_length(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def own_drift(p):
    d = np.linalg.norm(p - p[0], axis=1)
    return {"end_m": round(float(d[-1]), 4), "max_m": round(float(d.max()), 4),
            "path_length_m": round(path_length(p), 3)}  # fmt: skip


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--names", nargs=2, default=["A", "B"])
    ap.add_argument("--out")
    a = ap.parse_args()
    ta, pa = load(a.a)
    tb, pb = load(a.b)
    t0, t1 = max(ta[0], tb[0]), min(ta[-1], tb[-1])
    if t1 - t0 < 1.0:
        raise SystemExit(f"tracks overlap only {t1 - t0:.2f} s: are the stamps on the same clock?")
    keep = (ta >= t0) & (ta <= t1)
    ta, pa = ta[keep], pa[keep]
    pb_i = np.stack([np.interp(ta, tb, pb[:, k]) for k in range(3)], 1)
    pa_r, pb_r = pa - pa[0], pb_i - pb_i[0]
    R, t = umeyama(pb_r, pa_r)
    pb_al = pb_r @ R.T + t
    diff = np.linalg.norm(pa_r - pb_al, axis=1)
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    nA, nB = a.names
    report = {
        "overlap_s": round(float(ta[-1] - ta[0]), 1),
        "samples": int(len(ta)),
        "between": {
            "rms_m": round(float(np.sqrt((diff**2).mean())), 4),
            "max_m": round(float(diff.max()), 4),
            "final_m": round(float(diff[-1]), 4),
            "alignment_yaw_deg": round(yaw, 2),
        },
        nA: own_drift(pa_r),
        nB: own_drift(pb_r),
    }
    print(json.dumps(report, indent=2))
    if a.out:
        json.dump(report, open(a.out, "w"), indent=2)


if __name__ == "__main__":
    main()
