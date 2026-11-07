import numpy as np
from larp_autonomy.image_quality import metrics


def test_blank_scene_reports_no_trackable_texture():
    m=metrics(np.zeros((80,120),np.uint8))
    assert m['corners']==0 and m['feature_tile_fraction']==0 and m['dark_fraction']==1


def test_saturated_scene_is_detected():
    assert metrics(np.full((80,120),255,np.uint8))['saturated_fraction']==1
