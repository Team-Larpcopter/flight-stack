import numpy as np
from scipy.spatial.transform import Rotation
from larp_autonomy.map_global_tf import registration


def test_composition_includes_mount_rotation_and_translation():
    def transform(angles, translation):
        t = np.eye(4)
        t[:3, :3] = Rotation.from_euler('xyz', angles).as_matrix()
        t[:3, 3] = translation
        return t
    map_global = transform([0, 0, .7], [2, -3, .2])
    body_imu = transform([.4, -.2, 1.5], [.05, .02, -.01])
    for offset in (0, 3):
        global_imu = transform([.2, .3, -.4], [1+offset, 2, 3])
        map_body = map_global @ global_imu @ np.linalg.inv(body_imu)
        np.testing.assert_allclose(registration(map_body, body_imu, global_imu), map_global, atol=1e-12)
