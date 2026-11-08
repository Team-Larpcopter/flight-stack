"""Launcher checks without Docker, tmux sessions, or hardware."""
import os
from pathlib import Path
import subprocess


def test_loads_environment_and_blocks_competing_stack(tmp_path):
    source=Path(__file__).with_name('launch_system_tmux.sh')
    script=tmp_path/source.name
    script.write_text(source.read_text())
    (tmp_path/'deployment.env').write_text('export DEPLOYMENT_CONFIG=/config-from-env\n')
    for name,body in [('tmux','exit 1'),('docker','echo true')]:
        p=tmp_path/name;p.write_text('#!/bin/sh\n'+body+'\n');p.chmod(0o755)
    env={k:v for k,v in os.environ.items() if k!='DEPLOYMENT_CONFIG'}
    env['PATH']=str(tmp_path)+':'+env['PATH']
    result=subprocess.run(['bash',str(script),'hardware'],env=env,text=True,capture_output=True)
    assert result.returncode==2
    assert 'Stop flight-stack-takeoff-hold' in result.stderr
    assert 'DEPLOYMENT_CONFIG' not in result.stderr
