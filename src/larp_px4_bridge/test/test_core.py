import numpy as np
import pytest
from larp_px4_bridge.core import Converter, LatestSample, Sample, rotation


def sample(stamp=100):
    return Sample(stamp, [1,2,3], [1,0,0,0], [4,5,6], [1,2,3], np.eye(6), np.eye(6))


def test_flu_to_frd():
    out = Converter([0,1,0,0]).convert(sample())
    assert out['position'] == [1,-2,-3]
    assert out['velocity'] == [4,-5,-6]
    np.testing.assert_allclose(rotation(out['q']), np.eye(3), atol=1e-14)


def test_nonsymmetric_mount():
    mount = [np.sqrt(.5),0,0,np.sqrt(.5)]
    out = Converter(mount).convert(sample())
    np.testing.assert_allclose(out['velocity'], [-5,4,6], atol=1e-14)
    np.testing.assert_allclose(rotation(out['q']), np.diag([1,-1,-1]) @ rotation(mount).T, atol=1e-14)


def test_covariance_rotation():
    s = sample()
    s.twist_covariance[0,1] = s.twist_covariance[1,0] = .5
    out = Converter([np.cos(np.pi/8),0,0,np.sin(np.pi/8)]).convert(s)
    np.testing.assert_allclose(out['velocity_variance'], [.5,1.5,1])


def test_unknown_covariance():
    s = sample()
    s.pose_covariance = np.zeros((6,6))
    assert np.isnan(Converter([1,0,0,0]).convert(s)['position_variance']).all()


def test_fresh_once_and_monotonic():
    gate = LatestSample(20)
    assert gate.offer(sample(100),100)
    assert gate.offer(sample(101),101)
    assert gate.take(102).stamp_ns == 101
    assert gate.take(103) is None
    assert not gate.offer(sample(101),103)
    assert not gate.offer(sample(200),103)
    assert not gate.offer(sample(102),200)
    assert gate.offer(sample(201),201)
    assert gate.take(222) is None


def test_invalid_quaternion():
    with pytest.raises(ValueError):
        Converter([0,0,0,0])


@pytest.mark.parametrize('axis', range(3))
def test_source_quarter_turn(axis):
    s = sample()
    q = np.zeros(4)
    q[0] = np.sqrt(.5)
    q[axis+1] = np.sqrt(.5)
    s.quaternion = q
    out = Converter([0, 1, 0, 0]).convert(s)
    flip = np.diag([1, -1, -1])
    np.testing.assert_allclose(rotation(out['q']), flip @ rotation(q) @ flip, atol=1e-14)


@pytest.mark.parametrize('input_hz', [51, 200])
def test_latest_at_each_timer_tick(input_hz):
    gate = LatestSample(200_000_000)
    events = sorted([(1_000_000_000 + round(i*1e9/input_hz), 0) for i in range(input_hz)] +
                    [(1_000_000_000 + i*20_000_000, 1) for i in range(51)])
    delivered = []
    for stamp, kind in events:
        if kind == 0:
            gate.offer(sample(stamp), stamp)
        else:
            value = gate.take(stamp)
            if value is not None:
                delivered.append(value.stamp_ns)
                assert 0 <= stamp-value.stamp_ns <= 20_000_000
    assert 49 <= len(delivered) <= 51
    assert len(set(delivered)) == len(delivered)
    assert gate.take(2_100_000_000) is None
