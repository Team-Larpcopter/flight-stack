from flight_executive.extensions.tracking import Tracking
from flight_executive.model import Trajectory, Position


def test_certificate_expires_before_receipt_timeout_and_cannot_land():
    t=Tracking()
    t.accept(Trajectory(10.,(0,0,0),(0,0,0),(0,0,0),is_final=True,valid_until=10.1))
    assert t.sample(10.05,0) is not None
    assert t.sample(10.1,0) is None
    assert not t.complete(10.2,Position(10.2,0,0,0,0,0))


def test_nan_certificate_does_not_disable_expiry():
    t=Tracking()
    t.accept(Trajectory(10.,(0,0,0),(0,0,0),(0,0,0),valid_until=float('nan')))
    assert t.sample(10.01,0) is None
