#!/usr/bin/env python3
"""Safety check against test/fake_cm.py: a deactivated controller must come back on Ctrl-C, SIGTERM and the
--max-duration guard, and must stay off with --no-restore-on-exit. Exit 0 if every case behaves.

  ROS_DOMAIN_ID=87 python3 test/restore_check.py
"""
import os
import signal
import subprocess
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers

here = os.path.dirname(os.path.abspath(__file__))
root = os.path.dirname(here)
ENV = dict(os.environ, PYTHONPATH=os.pathsep.join([root, os.environ.get("PYTHONPATH", "")]))


def state(node, cli):
    fut = cli.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=3)
    return {c.name: c.state for c in fut.result().controller}["arm_controller"]


def wait_for(node, cli, want, timeout=15):
    end = time.time() + timeout
    while time.time() < end:
        if state(node, cli) == want:
            return True
        time.sleep(0.2)
    return False


def chaos(*args, code=None):
    cmd = [sys.executable, "-m", "armchaos.cli"] + list(args) + ["--source", "/joint_states_raw"]
    if code:
        cmd = [sys.executable, "-c", code]
    return subprocess.Popen(cmd, env=ENV, cwd=root)


GUARD = """
import sys
from armchaos.faults import make_fault
from armchaos.relay import run
from armchaos.scenario import Step
res = run([Step(1.0, 3600.0, make_fault('deactivate', controller='arm_controller'))],
          '/joint_states_raw', '/joint_states_chaos', '/controller_manager', max_duration=6.0, linger=0.5)
print('stopped_by', res.stopped_by, 'unrestored', res.unrestored)
sys.exit(0 if res.stopped_by == 'max-duration' and not res.unrestored else 1)
"""


def main():
    rclpy.init()
    node = rclpy.create_node("restore_check")
    cli = node.create_client(ListControllers, "/controller_manager/list_controllers")
    if not cli.wait_for_service(timeout_sec=10):
        print("start test/fake_cm.py first (same ROS_DOMAIN_ID)")
        return 2
    results = []

    def check(name, ok):
        results.append(ok)
        print(f"{'PASS' if ok else 'FAIL'}  {name}")

    for sig in (signal.SIGINT, signal.SIGTERM):
        p = chaos("inject", "deactivate", "controller=arm_controller", "--at", "1", "--for", "60")
        went_off = wait_for(node, cli, "inactive")
        p.send_signal(sig)
        try:
            p.wait(30)
        except subprocess.TimeoutExpired:
            p.kill()
        check(f"{sig.name}: controller deactivated, then restored, process exited {p.returncode}",
              went_off and wait_for(node, cli, "active", 3) and p.returncode == {signal.SIGINT: 130, signal.SIGTERM: 143}[sig])

    p = chaos(code=GUARD)
    went_off = wait_for(node, cli, "inactive")
    check("--max-duration guard: stopped by itself and restored", went_off and p.wait(40) == 0 and wait_for(node, cli, "active", 3))

    p = chaos("inject", "deactivate", "controller=arm_controller", "--at", "1", "--for", "60", "--no-restore-on-exit")
    went_off = wait_for(node, cli, "inactive")
    p.send_signal(signal.SIGINT)
    p.wait(30)
    check("--no-restore-on-exit leaves it off", went_off and state(node, cli) == "inactive")

    p = chaos("inject", "deactivate", "controller=arm_controller", "--at", "1", "--for", "3")
    p.wait(60)
    check(f"normal run: restored when the fault ends, exit {p.returncode}", p.returncode == 0 and state(node, cli) == "active")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
