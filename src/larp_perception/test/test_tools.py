"""Offline checks for the perception tools (no ROS graph, no camera)."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

import bench  # noqa: E402
import imu_six_face  # noqa: E402

G = 9.7996


def test_six_face_fit_recovers_scale_and_bias_with_tilted_faces():
    scale, bias = np.array([0.93, 0.94, 0.92]), np.array([0.05, -0.1, 0.2])
    dirs = [[0.1, 1, 0.15], [0.12, -1, 0.1], [1, 0.17, -0.1], [-1, 0.1, 0.15], [0.15, -0.1, 1], [0.1, 0.16, -1]]
    faces = []
    for v in dirs:
        true = G * np.array(v, float) / np.linalg.norm(v)
        mean = scale * true + bias
        ax = int(np.argmax(np.abs(mean)))
        faces.append({"axis": ax, "sign": "+" if mean[ax] > 0 else "-", "mean": mean.tolist()})
    s, b = imu_six_face.fit(faces, G)
    assert np.allclose(s, scale, atol=1e-4) and np.allclose(b, bias, atol=1e-4)


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
