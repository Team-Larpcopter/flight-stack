"""Capture live pinhole CameraInfo and TF; never substitute nominal mount geometry."""
import argparse,hashlib,json,time
from pathlib import Path
import numpy as np
import yaml


def opencv_yaml(data):
    """OpenCV FileStorage needs indented sequences and flow-style vector rows."""
    class Dumper(yaml.SafeDumper):
        def increase_indent(self, flow=False, indentless=False):
            return super().increase_indent(flow, False)
    def sequence(dumper, values):
        numeric=all(isinstance(x,(int,float)) for x in values)
        return dumper.represent_sequence('tag:yaml.org,2002:seq',values,flow_style=numeric)
    Dumper.add_representer(list,sequence)
    return '%YAML:1.0\n'+yaml.dump(data,Dumper=Dumper,sort_keys=False)


def transform_matrix(t):
    from scipy.spatial.transform import Rotation
    q=t.rotation;p=t.translation
    a=np.eye(4);a[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();a[:3,3]=[p.x,p.y,p.z]
    return a


def generate(out,infos,transforms,template_dir,body_frame,bench_only):
    for name, matrix in transforms.items():
        matrix=np.asarray(matrix)
        if (matrix.shape!=(4,4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3],[0,0,0,1],atol=1e-8)
            or not np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=1e-5)
            or not np.isclose(np.linalg.det(matrix[:3,:3]),1.,atol=1e-5)):
            raise ValueError('Invalid rigid transform: '+name)
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    if (out/'rig.json').exists():raise ValueError('Use a fresh calibration directory')
    cams={}
    for i,side in enumerate(('left','right')):
        ci=infos[side]
        if ci['model'] not in ('plumb_bob','') or any(abs(d)>1e-8 for d in ci['d']):raise ValueError('Pinhole undistorted input required')
        k=np.asarray(ci['k']).reshape(3,3)
        if not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0:raise ValueError('Invalid calibration')
        cams[f'cam{i}']={'T_imu_cam':transforms['T_imu_'+side].tolist(),'cam_overlaps':[1-i],
          'camera_model':'pinhole','distortion_model':'radtan','distortion_coeffs':[0.,0.,0.,0.],
          'intrinsics':[float(k[0,0]),float(k[1,1]),float(k[0,2]),float(k[1,2])],
          'resolution':[ci['width'],ci['height']],'rostopic':f'/autonomy/stereo/{side}/image'}
    baseline=np.linalg.norm(transforms['T_imu_left'][:3,3]-transforms['T_imu_right'][:3,3])
    if not .01<baseline<.5:raise ValueError('Implausible stereo baseline; inspect transform directions and units')
    (out/'kalibr_imucam_chain.yaml').write_text(opencv_yaml(cams))
    text=(Path(template_dir)/'kalibr_imu_chain.yaml').read_text()
    (out/'kalibr_imu_chain.yaml').write_text(text)
    # Keep estimator tuning identical to the selected template. Device geometry
    # is generated separately; no hidden tuning overrides are applied here.
    config=(Path(template_dir)/'estimator_config.yaml').read_text()
    (out/'estimator_config.yaml').write_text(config)
    rig={k:v.tolist() for k,v in transforms.items()}
    rig.update({'source':'Live undistorted CameraInfo and TF','body_frame':body_frame,'depth_frame':infos.get('depth',{}).get('frame',''),
      'input_camera_info':infos,'baseline_m':float(baseline),'bench_only':bench_only,
      'flight_qualified':False,'qualification_blockers':['IMU noise not hardware-characterized','Runtime latency and moving accuracy require validation']+
      (['Sensor-parent frame is a bench reference, not a surveyed airframe mount'] if bench_only else [])})
    (out/'rig.json').write_text(json.dumps(rig,indent=2)+'\n')
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file()}
    (out/'calibration-hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    return rig


def main():
    import rclpy,tf2_ros
    from sensor_msgs.msg import CameraInfo
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from ament_index_python.packages import get_package_share_directory
    ap=argparse.ArgumentParser();ap.add_argument('--out',required=True);ap.add_argument('--body-frame',default='base_link')
    ap.add_argument('--imu-frame',default='oak_imu_frame');ap.add_argument('--seconds',type=float,default=20.)
    ap.add_argument('--template-dir');ap.add_argument('--bench-only',action='store_true');ap.add_argument('--depth-info',default='')
    a=ap.parse_args()
    if a.body_frame!='base_link' and not a.bench_only:raise ValueError('A non-airframe reference is bench-only')
    rclpy.init();node=rclpy.create_node('autonomy_capture_calibration');buf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buf,node)
    infos={}
    def receive(m,key):infos[key]={'width':m.width,'height':m.height,'model':m.distortion_model,'d':list(m.d),'k':list(m.k),'p':list(m.p),'r':list(m.r),'frame':m.header.frame_id}
    for side in ('left','right'):
        node.create_subscription(CameraInfo,f'/autonomy/stereo/{side}/camera_info',lambda m,s=side:receive(m,s),qos_profile_sensor_data)
    if a.depth_info:node.create_subscription(CameraInfo,a.depth_info,lambda m:receive(m,'depth'),qos_profile_sensor_data)
    end=time.monotonic()+a.seconds;transforms={};last_error='Waiting for CameraInfo'
    try:
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            if any(s not in infos for s in ('left','right')) or (a.depth_info and 'depth' not in infos):continue
            try:
                transforms={f'T_imu_{s}':transform_matrix(buf.lookup_transform(a.imu_frame,infos[s]['frame'],Time()).transform) for s in ('left','right')}
                transforms['T_body_imu']=transform_matrix(buf.lookup_transform(a.body_frame,a.imu_frame,Time()).transform)
                if a.depth_info:
                    transforms['T_body_depth']=transform_matrix(buf.lookup_transform(a.body_frame,infos['depth']['frame'],Time()).transform)
                    # Depth is rectified: optical rays use P and rotate back through R.
                    r=np.array(infos['depth']['r']).reshape(3,3)
                    if not np.allclose(r.T@r,np.eye(3),atol=1e-5):raise ValueError('Invalid depth rectification')
                    transforms['T_body_depth'][:3,:3]=transforms['T_body_depth'][:3,:3]@r.T
                break
            except tf2_ros.TransformException as exc:last_error=str(exc);transforms={}
        if not transforms:raise RuntimeError('Live calibration unavailable: '+last_error)
        template=Path(a.template_dir) if a.template_dir else Path(get_package_share_directory('larp_localization'))/'config/oak_d_s2'
        rig=generate(a.out,infos,transforms,template,a.body_frame,a.bench_only)
        print(json.dumps({'baseline_m':rig['baseline_m'],'body_frame':a.body_frame,'flight_qualified':False}))
    finally:node.destroy_node();rclpy.try_shutdown()
