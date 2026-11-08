import math
import pytest
from larp_autonomy.mission_goal import goal_values


def test_goal_values_are_finite():
    assert goal_values('1', 2, 3, 0) == (1., 2., 3., 0.)
    for bad in (math.nan, math.inf, -math.inf):
        with pytest.raises(ValueError):
            goal_values(0, bad, 0, 0)
