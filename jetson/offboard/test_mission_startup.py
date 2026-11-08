"""Mission orchestration with fake Docker/tmux; never starts devices or ROS."""
import json
import os
from pathlib import Path
import subprocess
import pytest


def launcher(tmp_path, script, *args, fail_preflight=False):
    base=Path(__file__).parent
    for name in ('launch_system_tmux.sh','replace_stack.sh','launch_full_system_tmux.sh',
                 'launch_mission_tmux.sh','launch_ego_mission_tmux.sh','launch_ego_full_system_tmux.sh'):
        (tmp_path/name).write_text((base/name).read_text())
    (tmp_path/'mission.yaml').write_text('{}')
    (tmp_path/'deployment.env').write_text(f'export DEPLOYMENT_CONFIG={tmp_path}\n')
    (tmp_path/'runtime_mounts.sh').write_text('runtime_mounts=()\n')
    log=tmp_path/'calls.jsonl'
    fake='''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
name=Path(sys.argv[0]).name;args=sys.argv[1:]
with open(os.environ['CALL_LOG'],'a') as f:f.write(json.dumps([name,*args])+'\\n')
if name=='docker':
    if args[0]=='inspect':print('true')
    if args[0]=='run' and os.environ['FAIL_PREFLIGHT']=='1':sys.exit(9)
if name=='tmux' and args[0] in ('new-session','split-window'):print('%'+str(sum(1 for _ in open(os.environ['CALL_LOG']))))
'''
    for name in ('docker','tmux'):
        p=tmp_path/name;p.write_text(fake);p.chmod(0o755)
    env={k:v for k,v in os.environ.items() if k not in
         ('DEPLOYMENT_CONFIG','CONTAINER','SESSION','DDS_PROFILE','XRCE_REFS')}
    env.update(PATH=str(tmp_path)+':'+env['PATH'],CALL_LOG=str(log),
               FAIL_PREFLIGHT=str(int(fail_preflight)),PX4_TRANSPORT='udp4',ROSBAG_DIR=str(tmp_path/'bags'))
    result=subprocess.run(['bash',str(tmp_path/script),*args],env=env,text=True,capture_output=True)
    calls=[json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    return result,calls


@pytest.mark.parametrize('script,planner,mode',[
    ('launch_full_system_tmux.sh','super','shadow'),('launch_mission_tmux.sh','super','hardware'),
    ('launch_ego_full_system_tmux.sh','ego','shadow'),('launch_ego_mission_tmux.sh','ego','hardware')])
def test_full_mission_panes_and_backend_selection(tmp_path,script,planner,mode):
    result,calls=launcher(tmp_path,script)
    assert result.returncode==0,result.stderr
    panes=[c for c in calls if c[:2] in (['tmux','new-session'],['tmux','split-window'])]
    assert len(panes)==9
    commands=[c for c in calls if c[:2]==['tmux','send-keys'] and '-l' in c]
    assert len(commands)==9
    for c,part in zip(commands,('drivers','foxglove','vins','dds','depth','map','planner','executive','rosbag')):
        if part=='rosbag':
            assert 'record_bag.py' in c[-1]
        else:
            assert f'component:={part}' in c[-1] and f'planner:={planner}' in c[-1] and f'mode:={mode}' in c[-1]
    recorder=commands[-1][3]
    assert not any(c[:2]==['tmux','send-keys'] and c[3]==recorder and c[-1]=='Enter' for c in calls)
    assert any(c[-2:]==['mouse','on'] for c in calls)
    assert ['docker','stop','--time','3','flight-stack-mission-hardware'] in calls
    assert ['docker','stop','--time','3','flight-stack-mission-ego-hardware'] in calls
    assert ['tmux','kill-session','-t','flight-stack-ego-shadow'] in calls
    create=next(c for c in calls if c[:2]==['docker','create'])
    suffix='-ego' if planner=='ego' else ''
    assert create[create.index('--name')+1]==f'flight-stack-mission{suffix}-{mode}'
    assert '--runtime' in create and 'nvidia' in create
    assert not any('skybeta-ros-status' in c for c in calls)


def test_check_only_validates_without_replacing_or_starting_stack(tmp_path):
    result,calls=launcher(tmp_path,'launch_ego_mission_tmux.sh','--check')
    assert result.returncode==0,result.stderr
    assert 'planner=ego mode=hardware' in result.stdout
    assert len(calls)==1 and calls[0][:2]==['docker','run']
    assert '--network' in calls[0] and 'none' in calls[0]
    assert not any('--device' in x for x in calls[0])


def test_failed_preflight_never_starts_mission(tmp_path):
    result,calls=launcher(tmp_path,'launch_ego_mission_tmux.sh',fail_preflight=True)
    assert result.returncode!=0
    assert not any(c[:2] in (['docker','create'],['docker','start'],['tmux','new-session']) for c in calls)


def test_invalid_backend_fails_before_touching_stack(tmp_path):
    result,calls=launcher(tmp_path,'launch_system_tmux.sh','hardware','--planner','unknown')
    assert result.returncode==2 and not calls
