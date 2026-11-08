import math
import pytest
from larp_autonomy.mission_goal import goal_values


def test_goal_values_are_finite():
    assert goal_values('1', 2, 3, 0) == (1., 2., 3., 0.)
    for bad in (math.nan, math.inf, -math.inf):
        with pytest.raises(ValueError):
            goal_values(0, bad, 0, 0)


@pytest.mark.parametrize("wait_for_hold", [False, True])
def test_goal_publication_and_optional_hold_gate(monkeypatch, wait_for_hold):
    import rclpy
    from types import SimpleNamespace as M
    from builtin_interfaces.msg import Time
    from larp_autonomy import mission_goal
    published=[];callbacks={}
    class FakeNode:
        def declare_parameter(self,name,default):return M(value=wait_for_hold if name=='wait_for_hold' else default)
        def create_publisher(self,*args):return M(publish=published.append)
        def create_subscription(self,kind,topic,callback,qos):callbacks[topic]=callback
        def get_clock(self):return M(now=lambda:M(to_msg=lambda:Time()))
        def get_logger(self):return M(info=lambda text:None)
        def destroy_node(self):pass
    def spin(node):
        callbacks['/autonomy/map_state'](M(localization_epoch=1))
        if wait_for_hold:
            callbacks['/flight_executive/state'](M(data='TAKEOFF'))
            assert not published
            callbacks['/flight_executive/state'](M(data='HOLD'))
        else:
            assert '/flight_executive/state' not in callbacks
        assert len(published)==1 and published[0].localization_epoch==1
        callbacks['/autonomy/map_state'](M(localization_epoch=2))
        if wait_for_hold:
            callbacks['/flight_executive/state'](M(data='HOLD'))
        assert len(published)==2 and published[-1].localization_epoch==2
    monkeypatch.setattr(rclpy,'init',lambda:None)
    monkeypatch.setattr(rclpy,'create_node',lambda name:FakeNode())
    monkeypatch.setattr(rclpy,'spin',spin)
    monkeypatch.setattr(rclpy,'try_shutdown',lambda:None)
    mission_goal.main()
