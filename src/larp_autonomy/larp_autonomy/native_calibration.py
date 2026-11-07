"""Generate VIO calibration from a native SDK export, without a running TF tree."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
import yaml
from .calibration import generate,opencv_yaml


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--extrinsics',required=True);p.add_argument('--left',required=True);p.add_argument('--right',required=True)
    p.add_argument('--imu-calibration',required=True);p.add_argument('--out',required=True);p.add_argument('--template-dir',required=True)
    p.add_argument('--body-mount',help='JSON with surveyed T_body_imu, metres')
    p.add_argument('--bench-only',action='store_true');p.add_argument('--imu-hz',type=float,default=400)
    p.add_argument('--depth',help='depth geometry exported by oak_native_node (<factory>.depth.json)')
    a=p.parse_args()
    if not a.body_mount and not a.bench_only:raise ValueError('A surveyed body mount or explicit bench reference is required')
    extrinsics=json.loads(Path(a.extrinsics).read_text());transforms={};infos={}
    for side in ('left','right'):
        t=np.array(extrinsics['camera_to_imu_cm'][side],dtype=float);t[:3,3]*=.01
        transforms['T_imu_'+side]=t
        m=json.loads(Path(getattr(a,side)).read_text())
        k=m.get('K',m.get('intrinsics'));d=m.get('D')
        if d is None:raise ValueError('Image export must explicitly record distortion coefficients')
        infos[side]={'width':m['width'],'height':m['height'],'k':np.array(k).ravel().tolist(),
                     'd':d,'model':'plumb_bob','frame':'native_oak_'+side+'_optical'}
    if a.body_mount:transforms['T_body_imu']=np.array(json.loads(Path(a.body_mount).read_text())['T_body_imu'])
    else:
        # Bench origin is the left optical centre; x forward, y left, z up.
        optical_to_flu=np.eye(4);optical_to_flu[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
        transforms['T_body_imu']=optical_to_flu@np.linalg.inv(transforms['T_imu_left'])
    if a.depth:
        # On-device depth is aligned to the RECTIFIED left camera: r maps left optical
        # coordinates into that frame, so T_left_depth has rotation r^T and no translation.
        depth=json.loads(Path(a.depth).read_text())
        r=np.array(depth['r'],dtype=float).reshape(3,3)
        if not np.allclose(r.T@r,np.eye(3),atol=1e-5):raise ValueError('Invalid depth rectification')
        left_depth=np.eye(4);left_depth[:3,:3]=r.T
        transforms['T_body_depth']=transforms['T_body_imu']@transforms['T_imu_left']@left_depth
        infos['depth']={'width':depth['width'],'height':depth['height'],'k':depth['k'],'p':depth['p'],'r':depth['r'],
                        'd':[0.]*5,'model':'plumb_bob','frame':depth['frame']}
    rig=generate(a.out,infos,transforms,a.template_dir,'left_camera_flu' if not a.body_mount else 'base_link',a.bench_only)
    directory=Path(a.out)
    chain=directory/'kalibr_imucam_chain.yaml';text=chain.read_text()
    for side in ('left','right'):text=text.replace(f'/autonomy/stereo/{side}/image',f'/native_oak/{side}/image_undistorted')
    chain.write_text(text)
    imu=directory/'kalibr_imu_chain.yaml';cfg=yaml.safe_load(imu.read_text().split('\n',1)[1])
    cfg['imu0']['rostopic']='/autonomy/imu/data';cfg['imu0']['update_rate']=a.imu_hz
    imu.write_text(opencv_yaml(cfg))
    correction=json.loads(Path(a.imu_calibration).read_text())
    if correction['axes']!='raw':raise ValueError('Raw IMU axis correction required')
    config=directory/'estimator_config.yaml'
    import re
    config.write_text(re.sub(r'(?m)^gravity_mag:.*$',f"gravity_mag: {float(correction['g'])}",config.read_text()))
    rig['source']='DepthAI camera-to-IMU export in cm; per-frame undistorted projection; explicit raw-axis IMU correction'
    rig['image_timestamp']='paired mid-exposure';rig['imu_alignment']='bracketed acceleration at native gyro times; no extrapolation'
    inputs=[a.extrinsics,a.left,a.right,a.imu_calibration]+([a.body_mount] if a.body_mount else [])+([a.depth] if a.depth else [])
    rig['input_sha256']={str(x):hashlib.sha256(Path(x).read_bytes()).hexdigest() for x in inputs}
    (directory/'rig.json').write_text(json.dumps(rig,indent=2)+'\n')
    hashes={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in directory.iterdir() if f.is_file() and f.name!='calibration-hashes.json'}
    (directory/'calibration-hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    print(json.dumps({'baseline_m':rig['baseline_m'],'bench_only':rig['bench_only']}))
if __name__=='__main__':main()
