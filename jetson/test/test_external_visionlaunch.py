import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'external_visionlaunch.py'


class WorkerLifecycle(unittest.TestCase):
    def start(self, code, lease=0.4):
        process = subprocess.Popen([sys.executable, str(SCRIPT), '--worker'],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)
        process.stdin.write((json.dumps({'command': [sys.executable, '-c', code],
                                        'lease_seconds': lease}) + '\n').encode())
        process.stdin.flush()
        self.addCleanup(lambda: process.poll() is None and process.kill())
        return process

    def finish(self, process):
        process.wait(timeout=9)
        self.assertEqual(process.returncode, 0, process.stderr.read().decode())
        process.stdin.close()
        process.stdout.close()
        process.stderr.close()

    def test_normal_exit(self):
        self.finish(self.start('pass'))

    def test_stdin_eof_stops_owned_child(self):
        process = self.start('import time; print("ready", flush=True); time.sleep(60)')
        self.assertEqual(process.stdout.readline(), b'ready\n')
        process.stdin.close()
        self.finish(process)

    def test_expired_heartbeat_stops_owned_child(self):
        process = self.start('import time; print("ready", flush=True); time.sleep(60)')
        self.assertEqual(process.stdout.readline(), b'ready\n')
        self.finish(process)

    def test_resistant_descendant_is_killed_and_reaped(self):
        with tempfile.TemporaryDirectory() as directory:
            pidfile = Path(directory) / 'pid'
            child = ('import os,signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); '
                     'signal.signal(signal.SIGTERM,signal.SIG_IGN); '
                     f'open({str(pidfile)!r},"w").write(str(os.getpid())); time.sleep(60)')
            code = f'import subprocess,sys,time; subprocess.Popen([sys.executable,"-c",{child!r}]); time.sleep(60)'
            process = self.start(code, lease=1.0)
            deadline = time.monotonic() + 3
            while not pidfile.exists():
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.01)
            pid = int(pidfile.read_text())
            process.stdin.close()
            self.finish(process)
            with self.assertRaises(ProcessLookupError):
                os.kill(pid, 0)

    def test_child_does_not_consume_heartbeat(self):
        process = self.start('import sys; print(repr(sys.stdin.read()), flush=True)')
        self.assertEqual(process.stdout.readline(), b"''\n")
        self.finish(process)


if __name__ == '__main__':
    unittest.main()
