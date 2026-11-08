from flight_executive.envelope import Envelope


def test_operator_padding_expands_all_sides_without_removing_limits():
    e=Envelope([-4.1624,-5.5626,0],[1.9336,.5334,1.8],padding=.5)
    # Input NED y/z have opposite signs to map y/z at zero heading.
    assert e.contains((-4.2624,-.6334,-1.9))
    assert e.contains((2.0336,5.6626,.1))
    assert not e.contains((2.04,0,-.5))
    assert not e.contains((0,0,-1.91))
    assert e.contains((0,0,1.),True,.5)
    assert not e.contains((0,0,1.01),True,.5)
