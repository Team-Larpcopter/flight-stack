#!/usr/bin/env python3
"""Summarize native sensor timestamps without assuming requested sample rates."""
import argparse,csv,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('capture');a=p.parse_args()
rows=list(csv.DictReader(Path(a.capture).open()));streams={};out={}
for name in ('left','right','accel','gyro'):
    selected=[r for r in rows if r['stream']==name]
    if not selected:continue
    x={k:np.array([float(r[k]) for r in selected]) for k in selected[0] if k!='stream'}
    # Discard startup settling, and retain capture and arrival clocks separately.
    keep=x['arrival_ns']>x['arrival_ns'][0]+2e9
    if keep.sum()>10:x={k:v[keep] for k,v in x.items()}
    streams[name]=x;t=x['device_ns'];dt=np.diff(t)*1e-6
    out[name]={'samples':len(t),'capture_hz':float((len(t)-1)*1e9/(t[-1]-t[0])) if len(t)>1 else None,
      'interval_ms_p50_p95_max':np.percentile(dt,[50,95,100]).tolist() if len(dt) else [],
      'host_age_ms_p50_p95_p99_max':np.percentile((x['arrival_ns']-x['capture_ns'])*1e-6,[50,95,99,100]).tolist(),
      'sequence_gap_count':int(np.sum(np.diff(x['sequence'])>1)),'nonincreasing_device_time':int(np.sum(dt<=0))}
    if name in ('left','right'):out[name]['exposure_us_p50_p95_max']=np.percentile(x['exposure_us'],[50,95,100]).tolist()
    else:
        v=np.stack([x[k] for k in ('x','y','z')],axis=1)
        out[name].update(mean=v.mean(axis=0).tolist(),std=v.std(axis=0).tolist(),mean_norm=float(np.linalg.norm(v,axis=1).mean()))
if all(k in streams for k in ('accel','gyro')):
    at=streams['accel']['device_ns'];gt=streams['gyro']['device_ns'];i=np.searchsorted(at,gt);i=np.clip(i,1,len(at)-1)
    diff=np.minimum(np.abs(gt-at[i]),np.abs(gt-at[i-1]))
    out['imu_nearest_sample_offset_ms_p50_p95_max']=np.percentile(diff*1e-6,[50,95,100]).tolist()
if all(k in streams for k in ('left','right')):
    l,r=streams['left'],streams['right'];_,li,ri=np.intersect1d(l['sequence'],r['sequence'],return_indices=True)
    if len(li):
        out['stereo_matched_sequences']=len(li)
        end=(l['device_ns'][li]-r['device_ns'][ri])*1e-6
        mid=end-(l['exposure_us'][li]-r['exposure_us'][ri])*.0005
        out['stereo_end_skew_ms_p50_p95_max']=np.percentile(np.abs(end),[50,95,100]).tolist()
        out['stereo_mid_skew_ms_p50_p95_max']=np.percentile(np.abs(mid),[50,95,100]).tolist()
print(json.dumps(out,indent=2))
