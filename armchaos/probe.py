"""Watch the faulted stream with armdoc, so a run can check that the doctor named each fault.

Parent side (no ROS imports): ArmdocWatcher starts `python -m armchaos.probe`, which runs armdoc live with
/joint_states remapped to the faulted topic, and collects one (wall time, [codes]) entry per report.
Child side: `main` below."""
import importlib.util
import json
import os
import signal
import subprocess
import sys
import threading
import time


def parse_report_line(line):
    """One `armdoc --json` line -> list of codes, or None if the line is not a report."""
    line = line.strip()
    if not line.startswith("["):
        return None
    try:
        return [f["code"] for f in json.loads(line)]
    except (ValueError, KeyError, TypeError):
        return None


class ArmdocWatcher:
    def __init__(self, topic, period=1.0, grace=3.0, python=sys.executable):
        self.topic, self.period, self.grace, self.python = topic, period, grace, python
        self.reports = []          # [(wall_time, [codes])]
        self.other = []            # last non-report lines (errors), for diagnosing a dead probe
        self.proc = None

    @staticmethod
    def available():
        return importlib.util.find_spec("armdoc") is not None

    def start(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        env = dict(os.environ, PYTHONPATH=os.pathsep.join(p for p in (root, os.environ.get("PYTHONPATH", "")) if p))
        self.proc = subprocess.Popen(
            [self.python, "-m", "armchaos.probe", "--topic", self.topic, "--json",
             "--period", str(self.period), "--grace", str(self.grace)],
            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            codes = parse_report_line(line)
            if codes is None:
                self.other = (self.other + [line.rstrip()])[-5:]
            else:
                self.reports.append((time.time(), codes))

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def stop(self):
        if self.proc is None:
            return
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()


def main(argv=None):
    """Child: run armdoc with /joint_states remapped to the faulted topic. Unknown args go to armdoc."""
    import argparse
    ap = argparse.ArgumentParser(prog="armchaos.probe")
    ap.add_argument("--topic", default="/joint_states_chaos")
    a, rest = ap.parse_known_args(sys.argv[1:] if argv is None else argv)
    import rclpy
    from armdoc import doctor
    _init = rclpy.init
    rclpy.init = lambda **k: _init(args=["--ros-args", "-r", f"/joint_states:={a.topic}"])
    return doctor.main(rest)


if __name__ == "__main__":
    sys.exit(main())
