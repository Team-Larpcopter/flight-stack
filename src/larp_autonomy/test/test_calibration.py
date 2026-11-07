"""Geometry checks that need neither ROS nor a connected camera."""
from pathlib import Path
import json
import numpy as np
import pytest
import yaml
from larp_autonomy.calibration import generate
from larp_autonomy.stereo_frontend import scaled_calibration

TEMPLATES=Path(__file__).parents[2]/'larp_localization/config/oak_d_s2'


def fixture():
    k=[800.,0.,639.5,0.,800.,399.5,0.,0.,1.]
    info=dict(width=1280,height=800,model='plumb_bob',d=[0.]*5,k=k,frame='left')
    infos={side:dict(info,frame=side) for side in ('left','right','depth')}
    transforms={key:np.eye(4) for key in ('T_imu_left','T_imu_right','T_body_imu','T_body_depth')}
    transforms['T_imu_right'][0,3]=.075
    return infos,transforms


def test_native_resolution_keeps_factory_projection():
    k=fixture()[0]['left']['k'];p=np.c_[np.array(k).reshape(3,3),[0.,0.,0.]]
    scaled_k,scaled_p=scaled_calibration(k,p,1280,800,1280,800)
    np.testing.assert_array_equal(scaled_k,k)
    np.testing.assert_array_equal(scaled_p,p.ravel())


def test_optional_downsample_preserves_pixel_centre_geometry():
    k=fixture()[0]['left']['k'];p=np.c_[np.array(k).reshape(3,3),[-60.,0.,0.]]
    sk,sp=scaled_calibration(k,p,1280,800,640,400)
    ray=np.array([.2,-.1,1.])
    old=np.array(k).reshape(3,3)@ray
    new=np.array(sk).reshape(3,3)@ray
    np.testing.assert_allclose(new[:2],(old[:2]+.5)/2-.5)
    assert np.array(sp).reshape(3,4)[0,3]==-30.


def test_factory_transform_initializes_upstream_default_estimator(tmp_path):
    infos,transforms=fixture()
    rig=generate(tmp_path,infos,transforms,TEMPLATES,'oak_parent_frame',True)
    data=yaml.safe_load((tmp_path/'kalibr_imucam_chain.yaml').read_text().split('\n',1)[1])
    np.testing.assert_array_equal(data['cam1']['T_imu_cam'],transforms['T_imu_right'])
    assert data['cam0']['resolution']==[1280,800]
    cfg=(tmp_path/'estimator_config.yaml').read_text()
    upstream=TEMPLATES.parents[2]/'vendor/ov_srvins/config/euroc_mav/estimator_config.yaml'
    expected=upstream.read_text().replace('track_frequency: 21.0','track_frequency: 31.0')
    assert cfg.strip() == expected.strip()
    assert rig['bench_only'] and not rig['flight_qualified']
    assert json.loads((tmp_path/'calibration-hashes.json').read_text())['rig.json']


@pytest.mark.parametrize('bad', ['reflection','nan','scale','homogeneous'])
def test_invalid_extrinsics_rejected_before_config_write(tmp_path,bad):
    infos,transforms=fixture();t=transforms['T_body_imu']
    if bad=='reflection':t[0,0]=-1
    elif bad=='nan':t[0,3]=float('nan')
    elif bad=='scale':t[0,0]=2
    else:t[3,3]=0
    with pytest.raises(ValueError,match='Invalid rigid transform'):
        generate(tmp_path,infos,transforms,TEMPLATES,'base_link',False)
    assert not (tmp_path/'estimator_config.yaml').exists()


def test_distorted_input_not_relabelled_pinhole(tmp_path):
    infos,transforms=fixture();infos['left']['d']=[.1,0,0,0,0]
    with pytest.raises(ValueError,match='Pinhole'):
        generate(tmp_path,infos,transforms,TEMPLATES,'base_link',False)


def test_stereo_only_calibration_needs_no_depth(tmp_path):
    infos,transforms=fixture()
    del infos['depth'];del transforms['T_body_depth']
    rig=generate(tmp_path,infos,transforms,TEMPLATES,'oak_parent_frame',True)
    assert 'T_body_depth' not in rig
    assert rig['depth_frame']==''
    assert rig['baseline_m']==pytest.approx(.075)


def test_generated_chain_is_readable_by_opencv(tmp_path):
    import cv2
    infos,transforms=fixture()
    generate(tmp_path,infos,transforms,TEMPLATES,'oak_parent_frame',True)
    storage=cv2.FileStorage(str(tmp_path/'kalibr_imucam_chain.yaml'),cv2.FILE_STORAGE_READ)
    assert storage.isOpened()
    cam=storage.getNode('cam1')
    assert cam.getNode('intrinsics').size()==4
    assert cam.getNode('T_imu_cam').at(0).at(3).real()==pytest.approx(.075)
    storage.release()


def test_native_generator_requires_no_local_imu_correction(tmp_path, monkeypatch):
    from larp_autonomy.native_calibration import main
    infos, transforms = fixture()
    extrinsics = tmp_path/'extrinsics.json'
    matrices = {}
    for side in ('left', 'right'):
        matrix = transforms['T_imu_'+side].copy()
        matrix[:3, 3] *= 100
        matrices[side] = matrix.tolist()
        (tmp_path/(side+'.json')).write_text(json.dumps(dict(
            width=1280, height=800, intrinsics=np.array(infos[side]['k']).reshape(3,3).tolist(), D=[0.]*5)))
    extrinsics.write_text(json.dumps({'camera_to_imu_cm': matrices}))
    out = tmp_path/'generated'
    monkeypatch.setattr('sys.argv', ['native_calibration', '--extrinsics', str(extrinsics),
        '--left', str(tmp_path/'left.json'), '--right', str(tmp_path/'right.json'),
        '--out', str(out), '--template-dir', str(TEMPLATES), '--bench-only'])
    main()
    rig = json.loads((out/'rig.json').read_text())
    assert rig['imu_correction'] == 'none; biases estimated by sqrtVINS'
    assert len(rig['input_sha256']) == 3
    config = yaml.safe_load((out/'estimator_config.yaml').read_text().split('\n',1)[1])
    assert config['gravity_mag'] == 9.81
    imu = yaml.safe_load((out/'kalibr_imu_chain.yaml').read_text().split('\n',1)[1])
    assert imu['imu0']['rostopic'] == '/autonomy/imu/data'
    assert imu['imu0']['update_rate'] == 400


def test_bundled_camera_configuration_matches_native_raw_frames():
    import cv2
    chain = TEMPLATES/'kalibr_imucam_chain.yaml'
    cameras = yaml.safe_load(chain.read_text().split('\n',1)[1])
    matrices=[]
    for i,side in enumerate(('left','right')):
        cam=cameras[f'cam{i}']
        t=np.array(cam['T_imu_cam'])
        np.testing.assert_allclose(t[3],[0,0,0,1])
        np.testing.assert_allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-5)
        assert np.linalg.det(t[:3,:3]) == pytest.approx(1.,abs=1e-5)
        assert cam['rostopic'] == f'/native_oak/{side}/image_undistorted'
        assert cam['resolution'] == [1280,800]
        assert cam['distortion_coeffs'] == [0.,0.,0.,0.]
        matrices.append(t)
    assert np.linalg.norm(matrices[0][:3,3]-matrices[1][:3,3]) == pytest.approx(.07548695,abs=1e-7)
    storage=cv2.FileStorage(str(chain),cv2.FILE_STORAGE_READ)
    assert storage.isOpened()
    assert storage.getNode('cam0').getNode('T_imu_cam').size()==4
    storage.release()
    imu=yaml.safe_load((TEMPLATES/'kalibr_imu_chain.yaml').read_text().split('\n',1)[1])['imu0']
    assert imu['rostopic']=='/autonomy/imu/data'
    assert imu['update_rate']==400
