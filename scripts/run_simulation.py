"""Bounded integration run; signal launch only to avoid duplicate child SIGINTs."""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

folder = Path(sys.argv[1]).resolve()
folder.mkdir(parents=True, exist_ok=True)
with (folder/'launch.log').open('x') as log:
    proc = subprocess.Popen(['ros2', 'launch', 'rm_flatness_nav', 'mission.launch.py',
                             'sim:=true', f'output_dir:={folder}'], stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(24)
        if proc.poll() is not None:
            raise RuntimeError(f'Launch exited early: {proc.returncode}')
        proc.send_signal(signal.SIGINT)
        code = proc.wait(timeout=15)
        if code != 0:
            raise RuntimeError(f'Launch exit {code}; inspect launch.log')
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5)
print('PASS: bounded ROS launch and clean shutdown')
