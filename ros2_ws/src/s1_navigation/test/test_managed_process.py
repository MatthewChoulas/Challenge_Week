"""Exercise cleanup with real child processes without starting Gazebo."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

SUPERVISOR = Path(__file__).resolve().parents[1] / 's1_navigation/managed_process.py'


def wait_for(predicate, seconds=7):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError('Timed out waiting for process cleanup')


def alive(pid):
    try:
        # Orphan zombies are no longer running; the system reaper owns them.
        return Path(f'/proc/{pid}/stat').read_text().split(') ')[1][0] != 'Z'
    except FileNotFoundError:
        return False


def command(pid_file):
    script = (
        'import os, signal, subprocess, sys, time; '
        'signal.signal(signal.SIGINT, signal.SIG_IGN); '
        'signal.signal(signal.SIGTERM, signal.SIG_IGN); '
        'child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"]); '
        f'open({str(pid_file)!r}, "w").write(str(os.getpid()) + " " + str(child.pid)); '
        'time.sleep(60)'
    )
    return [sys.executable, str(SUPERVISOR), sys.executable, '-c', script]


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM, signal.SIGHUP])
def test_shutdown_cleans_up_child_and_descendant(tmp_path, sig):
    pid_file = tmp_path / 'pids'
    process = subprocess.Popen(command(pid_file), stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
    try:
        wait_for(lambda: pid_file.exists() and len(pid_file.read_text().split()) == 2)
        pids = list(map(int, pid_file.read_text().split()))
        process.send_signal(sig)
        process.wait(timeout=7)
        wait_for(lambda: all(not alive(pid) for pid in pids))
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
            process.wait(timeout=7)


def test_parent_disappearing_cleans_up_simulation(tmp_path):
    pid_file = tmp_path / 'pids'
    release = tmp_path / 'release'
    parent_code = (
        'import subprocess, time; from pathlib import Path; '
        f'p = subprocess.Popen({command(pid_file)!r}); '
        f'path = Path({str(release)!r})\n'
        'while not path.exists(): time.sleep(0.05)\n'
    )
    parent = subprocess.Popen([sys.executable, '-c', parent_code],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        wait_for(lambda: pid_file.exists() and len(pid_file.read_text().split()) == 2)
        pids = list(map(int, pid_file.read_text().split()))
        release.touch()
        parent.wait(timeout=3)
        wait_for(lambda: all(not alive(pid) for pid in pids))
    finally:
        release.touch()
        parent.wait(timeout=3)


def test_child_exit_cleans_up_remaining_descendant(tmp_path):
    pid_file = tmp_path / 'pids'
    args = command(pid_file)
    args[-1] = args[-1].rsplit('time.sleep(60)', 1)[0] + 'time.sleep(0.2)'
    process = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        wait_for(lambda: pid_file.exists() and len(pid_file.read_text().split()) == 2)
        pids = list(map(int, pid_file.read_text().split()))
        assert process.wait(timeout=7) == 0
        wait_for(lambda: all(not alive(pid) for pid in pids))
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
            process.wait(timeout=7)
