#!/usr/bin/env python3
"""Create an independent mission frame at old (1.2,0,0), yaw -pi/2.
Uses deployed assets; no rebuild, runtime search, or sensor extrinsic changes.
"""
import argparse
import math
from pathlib import Path
import shutil
import yaml


def transform(p):
    # p_new = R_old_to_new @ (p_old - origin_old).
    # Proper +90 degree rotation: determinant +1, no reflection.
    rotation = ((0., -1., 0.), (1., 0., 0.), (0., 0., 1.))
    relative = (float(p[0])-1.2, float(p[1]), float(p[2]))
    return [sum(row[j]*relative[j] for j in range(3)) for row in rotation]


def cfg_old_goal(source):
    return yaml.safe_load((source/"mission.yaml").read_text())["goal"][:3]


def prepare(bundle):
    bundle = Path(bundle)
    source = bundle/'config'
    dest = bundle/'config_new_start'
    shutil.copytree(source, dest, dirs_exist_ok=True)
    cfg = yaml.safe_load((source/'mission.yaml').read_text())
    install = bundle/'runtime/flight-stack/install'
    def asset(key, fallback):
        configured = cfg.get(key)
        if not configured:
            return install/fallback
        if configured.startswith('/config/'):
            return source/configured.removeprefix('/config/')
        if configured.startswith('/flight-stack/'):
            return bundle/'runtime'/configured.lstrip('/')
        return Path(configured)
    prior = asset('cad_prior_file', 'larp_navigation/share/larp_navigation/config/course_cad_prior_fine.txt')
    with prior.open() as inp, (dest/'course_cad_prior_new_start.txt').open('w') as out:
        for line in inp:
            cols = line.split()
            if not cols or cols[0].startswith('#') or cols[0]=='CAD_PRIOR_V1':
                out.write(line); continue
            xyz = transform(cols[:3])
            out.write(' '.join(f'{v:.9f}' for v in xyz)+' '+' '.join(cols[3:])+'\n')
    route = asset('follower_route_file', 'larp_route_follower/share/larp_route_follower/config/course_master_route.xyz')
    points = [list(map(float,l.split()[:3])) for l in route.read_text().splitlines() if l.strip() and not l.lstrip().startswith('#')]
    # Select a guide entry near this new mission origin; progress starts fresh at index zero.
    nearest = min(range(len(points)), key=lambda i:(points[i][0]-1.2)**2+points[i][1]**2)
    with (dest/'course_master_route_new_start.xyz').open('w') as out:
        out.write('# Progress guide in new start frame; live map determines collision-free motion.\n')
        for point in points[nearest:]:
            out.write(' '.join(f'{v:.9f}' for v in transform(point))+'\n')
    lower, upper = cfg['hard_boundary_lower'], cfg['hard_boundary_upper']
    cfg['hard_boundary_lower'] = [-upper[1], lower[0]-1.2, lower[2]]
    cfg['hard_boundary_upper'] = [-lower[1], upper[0]-1.2, upper[2]]
    cfg['goal'] = transform(cfg['goal'][:3])+[math.atan2(math.sin(cfg['goal'][3]+math.pi/2), math.cos(cfg['goal'][3]+math.pi/2))]
    cfg.update(initial_position_m=[0.,0.,0.], initial_yaw_rad=0., map_origin_ned=[0.,0.,0.], map_heading_ned=0.,
               mission_id=str(cfg['mission_id'])+'-new-start',
               hard_boundary_reference='Original course transformed to operator-specified start: old x=1.2 m, y=0, facing right 90 degrees.',
               cad_prior_file='/config/course_cad_prior_new_start.txt',
               follower_route_file='/config/course_master_route_new_start.xyz')
    (dest/'mission.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
    # Independent task definition, not an old course with completed segments.
    # Survey/reference coordinates are converted once to the new mission frame.
    task_points_old_frame = [[1.2,0.,1.], [1.2,-4.8261,1.],
                             [-3.3863,-4.8261,1.], [-3.3863,-.77724,1.],
                             cfg_old_goal(source)]
    fixed = {'waypoints': [transform(p) for p in task_points_old_frame]}
    (dest/'fixed_course.yaml').write_text(yaml.safe_dump(fixed, sort_keys=False))
    print('Prepared',dest,'goal:',cfg['goal'],'fixed waypoints:',fixed['waypoints'])


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle',type=Path)
    prepare(parser.parse_args().bundle)
