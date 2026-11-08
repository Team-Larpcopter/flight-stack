"""Exercise tmux orchestration without starting containers or accessing sensors."""
import json
import os
from pathlib import Path
import subprocess
import tempfile


def test_three_panes_and_mouse():
    script = Path(__file__).with_name('launch_localization.sh')
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        log = root / 'calls.jsonl'
        fake = '''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
with open(os.environ['CALL_LOG'], 'a') as f:
    f.write(json.dumps([Path(sys.argv[0]).name, *sys.argv[1:]]) + '\\n')
args = sys.argv[1:]
if Path(sys.argv[0]).name == 'docker':
    if args[:2] == ['container', 'inspect']: sys.exit(1)
    if 'inspect' in args: print('sha256:test-image')
if Path(sys.argv[0]).name == 'tmux' and args[0] == 'has-session': sys.exit(1)
'''
        for name in ('docker', 'tmux'):
            path = root / name
            path.write_text(fake)
            path.chmod(0o755)
        subprocess.run(['bash', str(script)], check=True, env={
            **os.environ, 'PATH': str(root) + ':' + os.environ['PATH'],
            'CALL_LOG': str(log), 'CONTAINER': 'test-flight', 'SESSION': 'test-flight'})
        calls = [json.loads(line) for line in log.read_text().splitlines()]
        panes = [c for c in calls if c[0] == 'tmux' and c[1] in ('new-session', 'split-window')]
        assert len(panes) == 3
        for pane, name in zip(panes, ('camera', 'vins', 'foxglove')):
            assert 'localization_pane.sh ' + name in pane[-1]
        assert ['tmux', 'set-option', '-t', 'test-flight', 'mouse', 'on'] in calls
        assert not any(c[:2] == ['docker', 'exec'] for c in calls)
