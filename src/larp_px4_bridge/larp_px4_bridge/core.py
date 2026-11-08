from dataclasses import dataclass
from typing import Sequence
import numpy as np


def unit(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('Expected a finite nonzero wxyz quaternion')
    norm = np.linalg.norm(q)
    if not np.isfinite(norm):
        raise ValueError('Quaternion magnitude overflow')
    return q / norm


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return np.array([w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
                     w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v])


def rotation(q):
    w, x, y, z = unit(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def variance(cov, transform):
    cov = np.asarray(cov, dtype=float).reshape(3, 3)
    if not np.any(cov):
        return [float('nan')] * 3
    return np.diag(transform @ cov @ transform.T).tolist()



@dataclass
class Sample:
    stamp_ns: int
    position: Sequence[float]
    quaternion: Sequence[float]
    velocity: Sequence[float]
    angular_velocity: Sequence[float]
    pose_covariance: Sequence[float]
    twist_covariance: Sequence[float]


class Converter:
    def __init__(self, mounting):
        self.mounting = unit(mounting)
        self.body = rotation(self.mounting)
        self.world = np.diag([1., -1., -1.])

    def convert(self, sample):
        vectors = np.asarray([sample.position, sample.velocity, sample.angular_velocity], dtype=float)
        if vectors.shape != (3, 3):
            raise ValueError('Expected three 3D vectors')
        pose = np.asarray(sample.pose_covariance).reshape(6, 6)
        twist = np.asarray(sample.twist_covariance).reshape(6, 6)
        inverse = self.mounting * [1, -1, -1, -1]
        q = multiply(multiply([0, 1, 0, 0], sample.quaternion), inverse)
        return dict(position=(self.world @ vectors[0]).tolist(), q=q.tolist(),
                    velocity=(self.body @ vectors[1]).tolist(),
                    angular_velocity=(self.body @ vectors[2]).tolist(),
                    position_variance=variance(pose[:3, :3], self.world),
                    orientation_variance=variance(pose[3:, 3:], self.body),
                    velocity_variance=variance(twist[:3, :3], self.body))


class LatestSample:
    def __init__(self, max_age_ns):
        self.max_age_ns = max_age_ns
        self.high_water = 0
        self.pending = None

    def offer(self, sample, now):
        if sample.stamp_ns == self.high_water or (self.max_age_ns > 0 and
                (sample.stamp_ns < self.high_water or not 0 <= now-sample.stamp_ns <= self.max_age_ns)):
            return False
        self.high_water = sample.stamp_ns
        self.pending = sample
        return True

    def take(self, now):
        sample, self.pending = self.pending, None
        if sample is not None and (self.max_age_ns == 0 or 0 <= now-sample.stamp_ns <= self.max_age_ns):
            return sample
        return None
