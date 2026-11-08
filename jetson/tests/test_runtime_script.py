"""Check host launch arguments with a fake Docker command; no devices opened."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


def fixture(tmp_path):
    root = tmp_path / 'checkout with spaces'
    (root / 'jetson').mkdir(parents=True)
    (root / 'install').mkdir()
    (root / 'install/setup.bash').touch()
    script = root / 'jetson/run_flight_stack.sh'
    shutil.copy(Path(__file__).parents[1] / 'run_flight_stack.sh', script)
    binary = tmp_path / 'bin'
    binary.mkdir()
    docker = binary / 'docker'
    docker.write_text(f'#!{sys.executable}\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n')
    docker.chmod(0o755)
    env = {k: v for k, v in os.environ.items() if not k.startswith('PX4_') and k not in
           ('DDS_PROFILE', 'BUILD_IMAGE', 'CONTAINER', 'ROS_DOMAIN_ID')}
    env['PATH'] = str(binary) + ':' + env['PATH']
    return root, script, env


def test_udp_maps_only_checkout_and_usb(tmp_path):
    root, script, env = fixture(tmp_path)
    result = subprocess.run(['bash', str(script), 'shadow', 'goal_x:=2.0'], env=env,
                            check=True, capture_output=True, text=True)
    args = json.loads(result.stdout)
    assert str(root) + ':/flight-stack' in args
    assert '--device' not in args and '--privileged' not in args
    assert 'agent_transport:=udp4' in args and 'mode:=shadow' in args
    assert 'goal_x:=2.0' in args


def test_invalid_transport_and_mode_override_fail_before_docker(tmp_path):
    _, script, env = fixture(tmp_path)
    for extra, settings in [(['mode:=hardware'], {}), ([], {'PX4_TRANSPORT': 'wrong'}),
                            ([], {'PX4_TRANSPORT': 'serial', 'PX4_DEVICE': '/missing/serial'})]:
        result = subprocess.run(['bash', str(script), 'shadow', *extra], env={**env, **settings},
                                capture_output=True, text=True)
        assert result.returncode != 0 and not result.stdout
