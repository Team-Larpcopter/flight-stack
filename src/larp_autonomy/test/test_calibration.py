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


def test_factory_transform_is_preserved_and_not_online_refit(tmp_path):
    infos,transforms=fixture()
    rig=generate(tmp_path,infos,transforms,TEMPLATES,'oak_parent_frame',True)
    data=yaml.safe_load((tmp_path/'kalibr_imucam_chain.yaml').read_text().split('\n',1)[1])
    np.testing.assert_array_equal(data['cam1']['T_imu_cam'],transforms['T_imu_right'])
    assert data['cam0']['resolution']==[1280,800]
    cfg=(tmp_path/'estimator_config.yaml').read_text()
    assert 'calib_cam_extrinsics: false' in cfg and 'calib_cam_intrinsics: false' in cfg
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


def test_native_depth_rig_uses_rectified_left_rotation(tmp_path,monkeypatch):
    import sys
    from larp_autonomy import native_calibration
    t_left=np.eye(4);t_left[:3,:3]=[[1,0,0],[0,-1,0],[0,0,-1]];t_left[:3,3]=[7.6,0,-.15]  # cm, as exported
    t_right=t_left.copy();t_right[0,3]=0.
    (tmp_path/'ext.json').write_text(json.dumps({'camera_to_imu_cm':{'left':t_left.tolist(),'right':t_right.tolist()}}))
    for side in ('left','right'):
        (tmp_path/f'{side}.json').write_text(json.dumps({'width':1280,'height':800,'D':[0.]*8,
            'intrinsics':[[800.,0,640],[0,800.,400],[0,0,1]]}))
    (tmp_path/'imu.json').write_text(json.dumps({'axes':'raw','accel_bias':[0,0,0],'accel_scale':[1,1,1],'g':9.8}))
    a=np.radians(.4);r=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
    (tmp_path/'depth.json').write_text(json.dumps({'frame':'native_oak_depth_optical','width':800,'height':500,
        'k':[500.,0,400,0,500.,250,0,0,1],'p':[500.,0,400,-37.75,0,500.,250,0,0,0,1,0],'r':r.ravel().tolist()}))
    out=tmp_path/'cal'
    monkeypatch.setattr(sys,'argv',['x','--extrinsics',str(tmp_path/'ext.json'),'--left',str(tmp_path/'left.json'),
        '--right',str(tmp_path/'right.json'),'--imu-calibration',str(tmp_path/'imu.json'),'--out',str(out),
        '--template-dir',str(TEMPLATES),'--bench-only','--depth',str(tmp_path/'depth.json')])
    native_calibration.main()
    rig=json.loads((out/'rig.json').read_text())
    assert rig['depth_frame']=='native_oak_depth_optical'
    t_imu_left=t_left.copy();t_imu_left[:3,3]*=.01
    left_depth=np.eye(4);left_depth[:3,:3]=r.T
    np.testing.assert_allclose(rig['T_body_depth'],np.array(rig['T_body_imu'])@t_imu_left@left_depth,atol=1e-9)
