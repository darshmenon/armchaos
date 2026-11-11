#!/usr/bin/env python3
"""Run a scenario against a live graph and record which armdoc code fires during each fault.

  e2e.py scenario.yaml --source /joint_states_raw [--out results.json]

Starts `armchaos run` and a remapped `armdoc` (live, period 1 s), waits, then prints a table. Run the
arm (fake_arm_raw.py, fake_cm.py or a simulator) separately, on the same ROS_DOMAIN_ID."""
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
import time

here = os.path.dirname(os.path.abspath(__file__))
root = os.path.dirname(here)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario")
    ap.add_argument("--source", default="/joint_states_raw")
    ap.add_argument("--out")
    ap.add_argument("--settle", type=float, default=4.0)
    a = ap.parse_args()
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([root, os.path.expanduser("~/armdoc"), os.environ.get("PYTHONPATH", "")]))
    ev = tempfile.NamedTemporaryFile(suffix=".jsonl", delete=False).name
    doc = subprocess.Popen([sys.executable, os.path.join(here, "armdoc_remap.py"), "--period", "1", "--grace", "3"],
                           env=env, stdout=subprocess.PIPE, text=True)
    time.sleep(3)                                   # armdoc sees the healthy relayed stream first
    chaos = subprocess.Popen([sys.executable, "-m", "armchaos.cli", "run", a.scenario, "--source", a.source,
                              "--events", ev], env=env, cwd=root)
    chaos.wait()
    time.sleep(a.settle)
    doc.terminate()
    lines = doc.communicate()[0].splitlines()
    events = [json.loads(l) for l in open(ev)]
    reports = []
    for l in lines:
        ts, _, rest = l.partition(" ")
        reports.append((float(ts), rest))
    rows = []
    starts, stops = {}, []
    for e in events:
        if e["event"] == "START":
            starts[e["label"]] = e
        else:
            stops.append((starts.pop(e["label"]), e))
    for k, (s, e) in enumerate(stops):
        nxt = stops[k + 1][0]["wall"] if k + 1 < len(stops) else float("inf")
        win = [(t, r) for t, r in reports if s["wall"] <= t <= e["wall"] + 3.0]
        codes = {}
        for t, r in win:
            m = re.match(r"(ERROR|WARN)\s+\[(\w+)\]", r)
            if m:
                codes.setdefault(m.group(2), round(t - s["wall"], 1))
        back = [r for t, r in reports if e["wall"] <= t <= min(nxt, e["wall"] + 6.0)]
        rows.append({"fault": s["label"], "codes": codes, "recovered": any(r.startswith("OK") for r in back)})
    print(f"{'fault':44} {'armdoc codes (seconds to first report)':52} recovered")
    for r in rows:
        c = ", ".join(f"{k} (+{v}s)" for k, v in r["codes"].items()) or "none"
        print(f"{r['fault']:44} {c:52} {'yes' if r['recovered'] else 'NO'}")
    if a.out:
        json.dump({"rows": rows, "events": events, "reports": reports}, open(a.out, "w"), indent=1)
    os.unlink(ev)


if __name__ == "__main__":
    main()
