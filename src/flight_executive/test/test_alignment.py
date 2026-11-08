from math import pi
import pytest
from flight_executive.alignment import GroundAlignment, from_pair, Alignment
from flight_executive.envelope import Envelope


def test_offset_floor_and_heading_preserve_course_geometry():
    expected=Alignment((2.,-3.,.403),.7)
    map_start=(.2,-.4,0.)
    ned=expected.to_ned(map_start)
    actual=from_pair(map_start,.3,ned,.4)
    assert actual.origin==pytest.approx(expected.origin)
    assert actual.heading==pytest.approx(.7)
    assert actual.to_ned((1.,2.,1.))==pytest.approx(expected.to_ned((1.,2.,1.)))
    envelope=Envelope([-4,-5,0],[2,.6,1.8],actual.heading,actual.origin)
    assert envelope.contains(actual.to_ned((0.,0.,0.)),ground_contact=True)
    assert envelope.contains(actual.to_ned((0.,0.,1.)))
    assert not envelope.contains(actual.to_ned((0.,0.,1.41)))
    assert not envelope.contains(actual.to_ned((0.,0.,-.01)),ground_contact=True)


def test_angle_wrap():
    assert from_pair((0,0,0),pi,(0,0,0),pi).heading==pytest.approx(0,abs=1e-12)


def test_freezes_once_after_stable_ground_window():
    g=GroundAlignment()
    for i in range(20):
        assert g.observe(i*.05,(0,0,0),0,(0,0,.403),0) is None
    a=g.observe(1.,(0,0,0),0,(0,0,.403),0)
    assert a.origin==(0.,0.,.403)
    assert g.observe(2.,(100,0,0),1,(200,0,0),2) is a


@pytest.mark.parametrize('change',['offset','yaw','motion','gap'])
def test_unstable_samples_restart_window(change):
    g=GroundAlignment()
    for i in range(20):g.observe(i*.05,(0,0,0),0,(0,0,0),0)
    t=1.5 if change=='gap' else 1.
    m=(.1,0,0) if change=='motion' else (0,0,0)
    p=(.1,0,0) if change in ('offset','motion') else (0,0,0)
    assert g.observe(t,m,0,p,.1 if change=='yaw' else 0) is None


def test_invalid_alignment_rejected():
    with pytest.raises(ValueError):from_pair((0,0,float('nan')),0,(0,0,0),0)
