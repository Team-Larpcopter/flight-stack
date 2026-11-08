import math
import numpy as np
from larp_autonomy.map_origin import Origin, Registration, StableOrigin


def test_one_shot_origin_waits_for_stability_and_does_not_rezero_motion():
    anchor=StableOrigin()
    assert anchor.observe(0.,(1,2,-.2),.4) is None
    assert anchor.observe(.2,(1,2,-.2),.4) is None
    assert anchor.observe(.4,(2,2,-.2),.4) is None
    for i in range(1,7):
        value=anchor.observe(.4+i*.2,(2,2,-.2),.4)
    assert value is not None
    assert anchor.observe(5.,(10,10,10),1.)==value


def test_stale_or_unhealthy_interval_restarts_stability_window():
    anchor=StableOrigin()
    anchor.observe(0.,(1,2,3),0.)
    assert anchor.observe(.5,(1,2,3),0.) is None
    anchor.reset_pending()
    assert anchor.observe(2.,(1,2,3),0.) is None
    assert anchor.value is None


def test_zero_offset_rotates_pose_and_covariance_consistently():
    r=Registration(Origin((1.,2.,-.2),math.pi/2),(0.,0.,0.),0.)
    np.testing.assert_allclose(r.position((1.,2.,-.2)),[0,0,0],atol=1e-12)
    np.testing.assert_allclose(r.position((1.,3.,-.2)),[1,0,0],atol=1e-12)
    q=(math.cos(math.pi/4),0.,0.,math.sin(math.pi/4))
    np.testing.assert_allclose(r.orientation(q),(1,0,0,0),atol=1e-12)
    covariance=np.diag([1.,2.,3.,4.,5.,6.])
    np.testing.assert_allclose(np.array(r.covariance(covariance.ravel())).reshape(6,6),
                              np.diag([2.,1.,3.,5.,4.,6.]),atol=1e-12)
