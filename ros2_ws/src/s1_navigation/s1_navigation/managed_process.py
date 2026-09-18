"""Run a launch child and clean up its process group on exit or parent loss."""
import os
import signal
import subprocess
import sys
import time


def run(command):
    parent_pid = os.getppid()
    stopping = False

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    previous = {sig: signal.signal(sig, stop)
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    child = None
    try:
        child = subprocess.Popen(command, start_new_session=True)
        while (not stopping and os.getppid() == parent_pid and
               child.poll() is None):
            time.sleep(0.1)
        return_code = child.poll()
        return 0 if return_code is None else return_code
    finally:
        if child is not None:
            # The launcher may have exited while its Gazebo descendants remain.
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(child.pid, sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic() + 1.0
                while time.monotonic() < deadline:
                    child.poll()  # Reap the direct child as soon as it exits.
                    try:
                        os.killpg(child.pid, 0)
                    except ProcessLookupError:
                        break
                    time.sleep(0.05)
                else:
                    continue
                break
            child.wait()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def main():
    if len(sys.argv) < 2:
        raise SystemExit('Usage: python -m s1_navigation.managed_process COMMAND [ARGS...]')
    raise SystemExit(run(sys.argv[1:]))


if __name__ == '__main__':
    main()
