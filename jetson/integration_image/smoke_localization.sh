#!/usr/bin/env bash
# Device-free runtime/linker/config check, inside the build container.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/sdk_install/lib:${LD_LIBRARY_PATH:-}"
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=187
python3 - <<'PY'
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
root = Path.cwd()
commands = {
    'camera': [str(root / 'install/larp_hardware/lib/larp_hardware/oak_native_node'),
               '--ros-args', '-p', 'connect_device:=false'],
    'sqrtvins': [str(root / 'install/ov_srvins/lib/ov_srvins/run_subscribe_msckf'),
                 '--ros-args', '-r', '__ns:=/fresh_clone_smoke', '-p',
                 'config_path:=' + str(root / 'src/larp_localization/config/oak_d_s2/estimator_config.yaml')],
}
for name, command in commands.items():
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        try:
            time.sleep(5)
            if process.poll() is not None:
                log.seek(0)
                raise RuntimeError(f'{name} exited during startup ({process.returncode}):\n{log.read()}')
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=10)
            if process.returncode not in (0, -signal.SIGINT):
                log.seek(0)
                raise RuntimeError(f'{name} failed shutdown ({process.returncode}):\n{log.read()}')
            print(f'{name}: device-free startup/config/shutdown passed')
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
PY
