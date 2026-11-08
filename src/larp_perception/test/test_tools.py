"""Offline checks for the perception tools (no ROS graph, no camera)."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

import bench  # noqa: E402



def test_depth_stats_fill_and_noise():
    rng = np.random.default_rng(0)
    frames = []
    for _ in range(30):
        d = np.full((50, 80), 2000.0) + rng.normal(0, 10, (50, 80))  # 2 m, 10 mm noise
        d[:, :20] = 0  # a quarter has no depth
        frames.append(d.astype(np.uint16))
    s = bench.depth_stats(frames)
    assert abs(s["fill_ratio"] - 0.75) < 1e-3
    assert abs(s["median_depth_m"] - 2.0) < 0.01
    assert 8 < s["temporal_noise_mm_median"] < 12


def test_stats_rate_and_gaps():
    s = bench.stats([0.0, 0.1, 0.2, 0.3, 0.5], [10.0, 20.0, 30.0])
    assert s["msgs"] == 5 and abs(s["hz"] - 8.0) < 1e-6 and s["gap_max_ms"] == 200.0
    assert s["age_median_ms"] == 20.0
