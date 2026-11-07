import math
import pytest
from larp_autonomy.trajectory_bridge import local_ned


def test_map_heading_and_translation_are_not_confused_with_enu():
    assert local_ned((1,2,3),0,(10,20,30))==(11,18,27)
    assert local_ned((1,2,3),math.pi/2)==pytest.approx((2,1,-3))
