"""Deployment configuration shared by all eight tmux panes."""
import math
from pathlib import Path
import yaml

COMPONENTS = ('drivers', 'foxglove', 'vins', 'dds', 'depth', 'map', 'super', 'executive')


def load_config(path, mode):
    if mode not in ('shadow', 'hardware'):
        raise ValueError('mode must be shadow or hardware')
    cfg = yaml.safe_load(Path(path).read_text())
    if not isinstance(cfg, dict):
        raise ValueError('mission configuration must be a mapping')
    for key, length in (('goal',4), ('hard_boundary_lower',3), ('hard_boundary_upper',3),
                        ('map_origin_ned',3), ('initial_position_m',3)):
        values = cfg[key]
        if len(values)!=length or not all(math.isfinite(float(v)) for v in values):
            raise ValueError('Invalid '+key)
        cfg[key] = [float(v) for v in values]
    for key in ('takeoff_height','map_heading_ned','initial_yaw_rad'):
        cfg[key] = float(cfg[key])
        if not math.isfinite(cfg[key]): raise ValueError('Invalid '+key)
    lo,hi = cfg['hard_boundary_lower'], cfg['hard_boundary_upper']
    # Deployment always caps the entire vehicle at 1.8 m; narrower boundaries win.
    hi[2] = min(hi[2],1.8)
    if lo[2]!=0.0 or any(b-a<=.8 for a,b in zip(lo,hi)):
        raise ValueError('Boundary requires floor z=0 and room for 0.4 m body margin')
    if not cfg.get('hard_boundary_reference'):
        raise ValueError('Boundary reference required')
    goal = cfg['goal'][:3]
    if not all(a+.4 <= v <= b-.4 for a,b,v in zip(lo,hi,goal)):
        raise ValueError('Goal violates body-clearance geofence')
    start = cfg['initial_position_m']
    if (not 0 < cfg['takeoff_height'] <= hi[2]-.4-start[2] or
        not all(lo[i]+.4 <= start[i] <= hi[i]-.4 for i in (0,1))):
        raise ValueError('Takeoff violates body-clearance geofence')
    if mode=='hardware' and (cfg.get('alignment_verified') is not True or not str(cfg.get('mission_id','')).strip()):
        raise ValueError('Hardware requires surveyed alignment_verified: true and mission_id')
    for key in ('rig_file', 'bridge_params') if mode=='hardware' else ('rig_file',):
        if not Path(cfg[key]).is_file(): raise ValueError('Missing '+key)
    rig = yaml.safe_load(Path(cfg['rig_file']).read_text())
    if not rig.get('T_body_depth') or rig.get('depth_frame')!='native_oak_depth_optical':
        raise ValueError('Rig requires calibrated rectified-depth transform')
    return cfg
