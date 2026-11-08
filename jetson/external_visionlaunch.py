#!/usr/bin/env python3
"""Four owned tmux workloads with container-side process-group cleanup."""
import argparse
import ctypes
import fcntl
import json
import math
import os
from pathlib import Path
import re
import select
import shlex
import signal
import subprocess
import sys
import time
import uuid


def cleanup(process, grace=2.0):
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            process.poll()
            if process.returncode is not None:
                try:
                    while os.waitpid(-1, os.WNOHANG)[0]:
                        pass
                except ChildProcessError:
                    pass
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.05)
        else:
            continue
        break
    process.wait()


def worker():
    # Docker exec disconnection does not reliably signal its container process.
    ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0)
    stopped = False

    def request_stop(*_):
        nonlocal stopped
        stopped = True

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, request_stop)
    payload = bytearray()
    while not payload.endswith(b'\n'):
        if stopped:
            return 0
        if not select.select([sys.stdin.fileno()], [], [], 5)[0]:
            return 2
        chunk = os.read(sys.stdin.fileno(), 1)
        if not chunk:
            return 2
        payload.extend(chunk)
        if len(payload) > 65536:
            raise ValueError('Worker configuration is too large')
    config = json.loads(payload)
    if stopped:
        return 0
    process = subprocess.Popen(config['command'], stdin=subprocess.DEVNULL, start_new_session=True)
    lease = float(config.get('lease_seconds', 3.0))
    last_heartbeat = time.monotonic()
    try:
        while not stopped and process.poll() is None:
            if select.select([sys.stdin.fileno()], [], [], 0.1)[0]:
                if not os.read(sys.stdin.fileno(), 4096):
                    break
                last_heartbeat = time.monotonic()
            if time.monotonic() - last_heartbeat > lease:
                print('Owner heartbeat expired; stopping this process group.', flush=True)
                break
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        cleanup(process)
    return max(0, process.returncode)


def pane(job_file):
    job = json.loads(Path(job_file).read_text())
    stopped = False

    def request_stop(*_):
        nonlocal stopped
        stopped = True

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, request_stop)
    if stopped:
        return 0
    process = subprocess.Popen(job['docker'], stdin=subprocess.PIPE, start_new_session=True)
    try:
        process.stdin.write((json.dumps({'command': job['command']}) + '\n').encode())
        process.stdin.flush()
        while not stopped and process.poll() is None:
            process.stdin.write(b'.')
            process.stdin.flush()
            time.sleep(0.5)
    except BrokenPipeError:
        pass
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        try:
            process.stdin.close()
        except BrokenPipeError:
            pass
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=3)
    return max(0, process.returncode)


def run(*args, check=True):
    return subprocess.run(args, check=check, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, timeout=15)


def env(name, default):
    return os.environ.get(name, default)


def source_command(command, install, extra=None, sdk=None):
    scripts = ['/opt/ros/humble/setup.bash', '/root/skybeta_ws/install/local_setup.bash',
               install + '/setup.bash']
    if extra:
        scripts.append(extra + '/local_setup.bash')
    lines = ['set -e'] + ['source ' + shlex.quote(p) for p in scripts]
    lines.append('export ROS_DOMAIN_ID=' + shlex.quote(env('ROS_DOMAIN_ID', '15')))
    if sdk:
        lines.append('export LD_LIBRARY_PATH=' + shlex.quote(sdk + '/lib') + ':${LD_LIBRARY_PATH:-}')
    lines.append('exec ' + shlex.join(command))
    return ['bash', '-lc', '\n'.join(lines)]


def attach(session):
    if launch_lock is not None:
        fcntl.flock(launch_lock, fcntl.LOCK_UN)
    command = 'switch-client' if os.environ.get('TMUX') else 'attach-session'
    return subprocess.call(['tmux', command, '-t', '=' + session])


def container_running(container):
    result = run('docker', 'inspect', '-f', '{{.State.Running}}', container, check=False)
    if result.returncode == 0:
        return result.stdout.strip() == 'true'
    if 'No such object' in result.stderr or 'No such container' in result.stderr:
        return False
    raise RuntimeError(f'Cannot verify container {container}: {result.stderr.strip()}')


def live_workers(state):
    live = []
    for container in state['containers']:
        if not container_running(container):
            continue
        top = run('docker', 'top', container, '-eo', 'pid,args')
        if state['helper'] in top.stdout:
            live.append(container)
    if container_running(state['agent']):
        live.append(state['agent'])
    return live


launch_lock = None


def launch(args):
    global launch_lock
    session = args.session
    if not re.fullmatch(r'[A-Za-z0-9_-]+', session):
        raise ValueError('Session must contain only letters, numbers, underscores, or hyphens')
    directory = Path(env('XDG_RUNTIME_DIR', '/tmp')) / f'flight-stack-ev-{os.getuid()}' / session
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    if args.action != 'status':
        launch_lock = (directory.parent / 'operation.lock').open('w')
        try:
            fcntl.flock(launch_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError('Another start or stop operation is in progress')
    state_path = directory / 'state.json'
    exists = run('tmux', 'has-session', '-t', '=' + session, check=False).returncode == 0
    if exists and args.action != 'status':
        recorded = json.loads(state_path.read_text()) if state_path.exists() else {}
        token = run('tmux', 'show-option', '-qv', '-t', session, '@external_vision_token').stdout.strip()
        if not token or recorded.get('token') != token:
            raise ValueError('Session is not owned by this launcher; refusing to attach or stop it')
    if args.action in ('stop', 'status'):
        state = json.loads(state_path.read_text()) if state_path.exists() else None
        if args.action == 'status':
            print(f'tmux session {session}: {"running" if exists else "absent"}')
            print('Owned workers:', ', '.join(live_workers(state)) if state else 'no saved state')
            return 0
        if exists:
            if state is None:
                raise ValueError('No ownership record for this session; refusing to kill it')
            run('tmux', 'kill-session', '-t', '=' + session)
        if state:
            deadline = time.monotonic() + 12
            while live_workers(state):
                if time.monotonic() >= deadline:
                    raise RuntimeError('Workers have not exited; inspect with status. No unrelated processes were killed.')
                time.sleep(0.25)
        print('External vision session stopped; owned container workers have exited.')
        return 0
    if exists:
        if not state_path.exists():
            raise ValueError('Session name is already in use without this launcher ownership record')
        return 0 if args.detach else attach(session)
    for other_state in directory.parent.glob('*/state.json'):
        if other_state == state_path:
            continue
        other = json.loads(other_state.read_text())
        other_session = other_state.parent.name
        token = run('tmux', 'show-option', '-qv', '-t', other_session,
                    '@external_vision_token', check=False).stdout.strip()
        if token and token == other.get('token'):
            raise ValueError(f'External vision session {other_session} already owns the stack; stop it first')
    if state_path.exists() and live_workers(json.loads(state_path.read_text())):
        raise ValueError('Previous workers are still stopping; run stop before starting again')
    calibration = args.calibration or os.environ.get('CALIBRATION_DIR')
    if not calibration:
        raise ValueError('Set CALIBRATION_DIR to the container calibration directory')
    mount = json.loads(env('IMU_TO_BODY_FRD', 'null'))
    if not isinstance(mount, list) or len(mount) != 4 or any(
            not isinstance(x, (int, float)) or not math.isfinite(x) for x in mount) or sum(x*x for x in mount) < 1e-16:
        raise ValueError('Set IMU_TO_BODY_FRD to the measured [w,x,y,z] mounting quaternion')
    mount = json.dumps([float(x) for x in mount])
    camera = env('CAMERA_CONTAINER', 'larp-localization-raw-imu')
    vio = env('VIO_CONTAINER', 'larp-localization-ready')
    containers = list(dict.fromkeys([camera, vio]))
    for container in containers:
        if run('docker', 'inspect', '-f', '{{.State.Running}}', container).stdout.strip() != 'true':
            raise ValueError(f'Container {container} must already be running')
    active = run('docker', 'ps', '--format', '{{.Names}}').stdout.splitlines()
    for container in active:
        top = run('docker', 'top', container, '-eo', 'pid,args').stdout
        if re.search(r'external-vision-[0-9a-f]+\.py --worker', top):
            raise ValueError(f'An external-vision worker already runs in {container}; stop its owner first')
        for executable in ('oak_native_node', 'run_subscribe_msckf', 'vins_px4_bridge', 'MicroXRCEAgent'):
            if re.search(r'(?<![\w])' + executable + r'(?![\w])', top):
                raise ValueError(f'{executable} already runs in {container}; stop its owner first')
    device = env('XRCE_DEV', '/dev/ttyTHS1')
    refs = str(Path(env('XRCE_REFS', '~/skybeta/host/dds/agent_refs_local.xml')).expanduser().resolve())
    if not Path(device).exists() or not Path(refs).is_file():
        raise ValueError(f'UART or DDS reference file missing: {device}, {refs}')
    install = env('INSTALL_PREFIX', '/integration/install_clean')
    bridge_install = env('BRIDGE_INSTALL_PREFIX', '/integration/install_px4_bridge')
    for path in (calibration + '/estimator_config.yaml', bridge_install + '/local_setup.bash'):
        run('docker', 'exec', vio, 'test', '-r', path)
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    token = uuid.uuid4().hex[:12]
    helper = f'/tmp/external-vision-{token}.py'
    script = str(Path(__file__).resolve())
    for container in containers:
        run('docker', 'cp', script, container + ':' + helper)
    agent = f'larp-ev-agent-{token}'
    camera_command = ['ros2', 'run', 'larp_hardware', 'oak_native_node', '--ros-args',
                      '-p', 'connect_device:=true', '-p', 'publish_images:=true',
                      '-p', 'align_imu:=true', '-p', 'accel_hz:=500', '-p', 'gyro_hz:=400',
                      '-p', 'max_exposure_us:=5000', '-p', 'capture_seconds:=0.0']
    jobs = [
        ('Camera + IMU', camera, source_command(camera_command, env('CAMERA_INSTALL_PREFIX', install),
                                               sdk=env('SDK_PREFIX', '/integration/sdk_install'))),
        ('sqrtVINS', vio, source_command(['ros2', 'run', 'ov_srvins', 'run_subscribe_msckf', '--ros-args',
            '-r', '__ns:=/autonomy', '-r', '/sim/processed/vio:=/autonomy/processed_vio',
            '-p', 'config_path:=' + calibration + '/estimator_config.yaml', '-p', 'verbosity:=INFO'], install)),
        ('MicroXRCE-DDS', None, ['MicroXRCEAgent', 'serial', '--dev', device, '-b', env('XRCE_BAUD', '921600'),
                                '-r', '/etc/agent_refs.xml', '-v', '4']),
        ('External vision', vio, source_command(['ros2', 'run', 'larp_px4_bridge', 'vins_px4_bridge', '--ros-args',
            '-p', 'imu_to_body_frd_quaternion_wxyz:=' + mount, '-p', 'publish_rate_hz:=50.0',
            '-p', 'input_topic:=/autonomy/odomimu', '-p', 'output_topic:=/fmu/in/vehicle_visual_odometry'],
            install, extra=bridge_install)),
    ]
    state_path.write_text(json.dumps({'containers': containers, 'helper': helper, 'agent': agent, 'token': token}))
    first = None
    try:
        for index, (title, container, command) in enumerate(jobs):
            if container:
                docker = ['docker', 'exec', '-i', container, 'python3', helper, '--worker']
            else:
                docker = ['docker', 'run', '--rm', '-i', '--name', agent, '--network', 'host',
                    '--device', device + ':' + device, '-v', refs + ':/etc/agent_refs.xml:ro',
                    '-v', script + ':' + helper + ':ro', '--entrypoint', 'python3',
                    env('AGENT_IMAGE', 'larp-integration:20261107'), helper, '--worker']
            job_file = directory / f'{index}.json'
            job_file.write_text(json.dumps({'docker': docker, 'command': command}))
            pane_command = shlex.join([sys.executable, script, '--pane', str(job_file)])
            if first is None:
                first = run('tmux', 'new-session', '-d', '-s', session, '-n', 'external-vision',
                            '-P', '-F', '#{pane_id}', 'sleep infinity').stdout.strip()
                run('tmux', 'set-option', '-t', session, '@external_vision_token', token)
                target = first
                run('tmux', 'set-option', '-w', '-t', first, 'remain-on-exit', 'on')
                run('tmux', 'set-option', '-w', '-t', first, 'pane-border-status', 'top')
                run('tmux', 'set-option', '-w', '-t', first, 'pane-border-format', ' #{pane_title} ')
                run('tmux', 'respawn-pane', '-k', '-t', first, pane_command)
            else:
                target = run('tmux', 'split-window', '-t', first, '-P', '-F', '#{pane_id}', pane_command).stdout.strip()
            run('tmux', 'select-pane', '-t', target, '-T', title)
            run('tmux', 'select-layout', '-t', first, 'tiled')
    except BaseException:
        if first:
            run('tmux', 'kill-session', '-t', '=' + session, check=False)
        raise
    print(f'Started {session}. Ctrl+C stops one pane. Use this script with stop to stop all four.')
    return 0 if args.detach else attach(session)


def main():
    if sys.argv[1:2] == ['--worker']:
        return worker()
    if sys.argv[1:2] == ['--pane']:
        return pane(sys.argv[2])
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start', 'stop', 'status'], nargs='?', default='start')
    parser.add_argument('--calibration', help='Calibration directory inside the VINS container')
    parser.add_argument('--session', default=env('SESSION', 'external_visionlaunch'))
    parser.add_argument('--detach', action='store_true', help='Start without attaching')
    args = parser.parse_args()
    try:
        return launch(args)
    except (ValueError, RuntimeError, subprocess.SubprocessError, OSError) as error:
        print(f'error: {error}', file=sys.stderr)
        if isinstance(error, subprocess.CalledProcessError) and error.stderr:
            print(error.stderr.strip(), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
