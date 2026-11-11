#!/usr/bin/env python3
"""armchaos: break a ros2_control arm on purpose, then restore it."""
import argparse
import sys

from armchaos.faults import FAULTS, make_fault
from armchaos.scenario import Step, parse_scenario


def _kv(items):
    out = {}
    for it in items:
        if "=" not in it:
            raise ValueError(f"parameter {it!r} must look like key=value")
        k, v = it.split("=", 1)
        try:
            out[k] = float(v)
        except ValueError:
            out[k] = v
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(prog="armchaos", description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="list the faults and their parameters")
    for name in ("run", "inject"):
        p = sub.add_parser(name, help="run a scenario file" if name == "run" else "apply one fault")
        if name == "run":
            p.add_argument("scenario")
        else:
            p.add_argument("fault", choices=sorted(FAULTS))
            p.add_argument("params", nargs="*", help="fault parameters as key=value (see `armchaos list`)")
            p.add_argument("--at", type=float, default=2.0, help="seconds before the fault starts")
            p.add_argument("--for", dest="dur", type=float, default=8.0, help="seconds the fault lasts")
        p.add_argument("--source", default="/joint_states", help="real joint states to relay")
        p.add_argument("--out", default="/joint_states_chaos", help="topic the faulted stream is published on")
        p.add_argument("--cm", default="/controller_manager")
        p.add_argument("--events", help="write start/stop events here as JSON lines")
        p.add_argument("--dry-run", action="store_true", help="print the plan, touch nothing")
    a = ap.parse_args(argv)

    if a.cmd == "list":
        for n, c in sorted(FAULTS.items()):
            d = ", ".join(f"{k}={v}" for k, v in c().defaults().items())
            print(f"{n:11} {c.help}\n{'':11} params: {d or '-'}")
        return 0
    try:
        if a.cmd == "run":
            import yaml
            with open(a.scenario) as f:
                steps = parse_scenario(yaml.safe_load(f))
        else:
            steps = [Step(a.at, a.dur, make_fault(a.fault, **_kv(a.params)))]
    except (ValueError, KeyError, OSError) as e:
        print(f"armchaos: {e}", file=sys.stderr)
        return 2
    for s in steps:
        print(f"plan: t={s.at:g}s for {s.dur:g}s  {s.label()}")
    print(f"plan: relay {a.source} -> {a.out}" + (f", controllers via {a.cm}" if any(s.fault.controller for s in steps) else ""))
    if a.dry_run:
        return 0
    from armchaos.relay import run
    run(steps, a.source, a.out, a.cm, a.events)
    return 0


if __name__ == "__main__":
    sys.exit(main())
