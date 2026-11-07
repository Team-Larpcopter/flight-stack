"""Checks for the calibration path that feeds sqrtVINS."""

import json
import os
from importlib.machinery import SourceFileLoader

import cv2
import numpy as np

HERE = os.path.dirname(__file__)
CALIB = os.path.join(HERE, "..", "config", "oak_d_s2", "oak_calib_skybeta15.json")
TOOL = SourceFileLoader("oak_to_kalibr", os.path.join(HERE, "..", "tools", "oak_to_kalibr.py")).load_module()


def load():
    return json.load(open(CALIB))


def test_opencv_uses_the_oak_rational_order():
    """larp_undistort relies on OpenCV reading depthai's 8 coefficients in the same order."""
    c = load()
    for side in ("left", "right"):
        K, d = np.array(c[side]["K"]), np.array(c[side]["D"])
        x, y = np.meshgrid(np.linspace(-0.7, 0.7, 11), np.linspace(-0.45, 0.45, 7))
        x, y = x.ravel(), y.ravel()
        ours = np.stack(TOOL.rational_distort(x, y, d), 1) * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
        pts = np.stack([x, y, np.ones_like(x)], 1)
        cv, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), K, d)
        assert np.abs(cv[:, 0, :] - ours).max() < 1e-6


def test_undistortion_map_recovers_pinhole_projection():
    """A point drawn with the full lens model lands at its pinhole position after remap."""
    c = load()
    K, d = np.array(c["left"]["K"]), np.array(c["left"]["D"])
    w, h = c["left"]["size"]
    m1, m2 = cv2.initUndistortRectifyMap(K, d, None, K, (w, h), cv2.CV_32FC1)
    for x, y in ((0.3, 0.2), (-0.5, 0.3), (0.6, -0.35)):
        u_pin, v_pin = K[0, 0] * x + K[0, 2], K[1, 1] * y + K[1, 2]
        xd, yd = TOOL.rational_distort(np.array([x]), np.array([y]), d)
        u_raw, v_raw = K[0, 0] * xd[0] + K[0, 2], K[1, 1] * yd[0] + K[1, 2]
        # the map says which raw pixel each undistorted pixel samples
        assert abs(m1[int(round(v_pin)), int(round(u_pin))] - u_raw) < 1.0
        assert abs(m2[int(round(v_pin)), int(round(u_pin))] - v_raw) < 1.0


def test_radtan_fit_is_not_good_enough():
    """Documents why larp_undistort exists: radtan cannot model this lens."""
    c = load()
    _, err = TOOL.fit_radtan(np.array(c["left"]["K"]), c["left"]["D"], c["left"]["size"])
    assert err["mean_px"] > 0.5


def test_generated_configs_are_undistorted_and_stereo():
    cfg = os.path.join(HERE, "..", "config", "oak_d_s2")
    imucam = open(os.path.join(cfg, "kalibr_imucam_chain.yaml")).read()
    assert imucam.count("rostopic: /oak/") == 2 and "image_undistorted" in imucam
    assert "distortion_coeffs: [0.000000000, 0.000000000, 0.000000000, 0.000000000]" in imucam
    est = open(os.path.join(cfg, "estimator_config.yaml")).read()
    assert "max_cameras: 2" in est and "use_stereo: true" in est
