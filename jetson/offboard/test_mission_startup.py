"""Launcher cleanup without Docker, tmux sessions, or hardware."""
import os
from pathlib import Path
import subprocess


def test_loads_environment_and_replaces_competing_stack(tmp_path):
    base=Path(__file__).parent
    for name in ('launch_system_tmux.sh','replace_stack.sh'):
        (tmp_path/name).write_text((base/name).read_text())
    (tmp_path/'deployment.env').write_text('export DEPLOYMENT_CONFIG=/config-from-env\n')
    # Stop before real configuration/container creation.
    (tmp_path/'runtime_mounts.sh').write_text('exit 0\n')
    log=tmp_path/'calls'
    for name in ('tmux','docker'):
        p=tmp_path/name
        p.write_text('#!/bin/sh\nprintf "%s\\n" "'+name+' $*" >> "$CALL_LOG"\n'
                     'case "$*" in *"State.Running"*) echo true;; esac\n')
        p.chmod(0o755)
    env={k:v for k,v in os.environ.items() if k!='DEPLOYMENT_CONFIG'}
    env.update(PATH=str(tmp_path)+':'+env['PATH'],CALL_LOG=str(log))
    result=subprocess.run(['bash',str(tmp_path/'launch_system_tmux.sh'),'hardware'],
                          env=env,text=True,capture_output=True)
    assert result.returncode==0,result.stderr
    calls=log.read_text()
    assert 'docker stop --time 3 flight-stack-takeoff-hold' in calls
    assert 'docker rm flight-stack-mission-hardware' in calls
    assert 'tmux kill-session -t flight-stack-hardware' in calls
    assert 'skybeta-ros-status' not in calls
    assert 'attach-session' not in calls
