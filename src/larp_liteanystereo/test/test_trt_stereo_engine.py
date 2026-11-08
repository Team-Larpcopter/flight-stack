import numpy as np
from larp_liteanystereo.trt_stereo_engine import (
    disparity_to_depth_mm,
)


def test_disparity_to_depth_mm_close_range():
    # At 1280x800 with fx=800, baseline=0.075m:
    # At network size 480x320, fx_net = 800 * (480 / 1280) = 300 px.
    # For a close object at Z = 0.25 m (250 mm), disparity = 300 * 0.075 / 0.25 = 90 px.
    disp = np.full((320, 480), 90.0, dtype=np.float32)
    depth_mm, scale_x = disparity_to_depth_mm(
        disp,
        fx_orig=800.0,
        baseline_m=0.075,
        orig_w=1280,
        orig_h=800,
        min_depth_m=0.15,
        max_depth_m=8.0,
        remove_invisible=True,
    )
    assert np.isclose(scale_x, 480.0 / 1280.0)
    # Left border < 90 px should be masked as non-overlapping (0 mm)
    assert np.all(depth_mm[:, :90] == 0)
    # Valid overlapping region (>= 90 px) should be 250 mm
    assert np.all(depth_mm[:, 90:] == 250)
