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


def test_full_system_eight_panes_and_separate_mission_modes():
    base=Path(__file__).parent
    for script,mode in [('launch_full_system_tmux.sh','shadow'),('launch_mission_tmux.sh','hardware')]:
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);log=root/'calls.jsonl'
            (root/'mission.yaml').write_text('{}')
            fake='''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
name=Path(sys.argv[0]).name;args=sys.argv[1:]
with open(os.environ['CALL_LOG'],'a') as f:f.write(json.dumps([name,*args])+'\\n')
if name=='docker' and args[:2]==['container','inspect']:sys.exit(1)
if name=='tmux' and args[0]=='has-session':sys.exit(1)
if name=='tmux' and args[0] in ('new-session','split-window'):print('%0')
'''
            for name in ('docker','tmux'):
                p=root/name;p.write_text(fake);p.chmod(0o755)
            subprocess.run(['bash',str(base/script)],check=True,env={**os.environ,
                'PATH':str(root)+':'+os.environ['PATH'],'CALL_LOG':str(log),'DEPLOYMENT_CONFIG':str(root),
                'SESSION':'test-flight','CONTAINER':'test-flight','PX4_TRANSPORT':'udp4'})
            calls=[json.loads(line) for line in log.read_text().splitlines()]
            panes=[c for c in calls if c[0]=='tmux' and c[1] in ('new-session','split-window')]
            assert len(panes)==8
            for c,part in zip(panes,('drivers','foxglove','vins','dds','depth','map','super','executive')):
                assert 'component:='+part in c[-1]
                assert 'mode:='+mode in c[-1]
            assert ['tmux','set-option','-t','test-flight','mouse','on'] in calls
            assert any('--runtime' in c and 'nvidia' in c for c in calls)
