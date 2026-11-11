#!/usr/bin/env python3
"""armchaos: break a ros2_control arm on purpose, then restore it.

Exit codes: 0 ok, 1 an `expect:` check failed, 2 bad usage, 3 a controller could not be restored,
130/143 stopped by Ctrl-C / SIGTERM (controllers are restored first)."""
import argparse
import sys

from armchaos import __version__
from armchaos.faults import FAULTS, make_fault
from armchaos.probe import ArmdocWatcher
from armchaos.report import build_results, exit_code, format_table
from armchaos.scenario import Step, over_budget, parse_expect, parse_scenario


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


def build_parser():
    ap = argparse.ArgumentParser(prog="armchaos", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--version", action="version", version=f"armchaos {__version__}")
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
            p.add_argument("--expect", action="append", metavar="CODE",
                           help="armdoc code that must fire (repeat for several, or `none`); needs --armdoc")
        p.add_argument("--source", default="/joint_states", help="real joint states to relay")
        p.add_argument("--out", default="/joint_states_chaos", help="topic the faulted stream is published on")
        p.add_argument("--cm", default="/controller_manager")
        p.add_argument("--events", help="write start/stop events here as JSON lines")
        p.add_argument("--dry-run", action="store_true", help="print the plan, touch nothing")
        p.add_argument("--armdoc", action="store_true",
                       help="run armdoc on the faulted stream and check each step's `expect:` codes")
        p.add_argument("--armdoc-lead", type=float, default=4.0,
                       help="healthy seconds before the first fault, so armdoc sees a good baseline (with --armdoc)")
        p.add_argument("--max-duration", type=float, default=600.0, metavar="SECONDS",
                       help="hard limit for the whole run; a longer scenario is refused (default 600)")
        p.add_argument("--restore-on-exit", action=argparse.BooleanOptionalAction, default=True,
                       help="on Ctrl-C, SIGTERM or the --max-duration guard, switch deactivated controllers "
                            "back on (default on)")
    return ap


def main(argv=None):
    a = build_parser().parse_args(argv)
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
            steps = [Step(a.at, a.dur, make_fault(a.fault, **_kv(a.params)),
                          expect=parse_expect(a.expect if a.expect != ["none"] else "none") if a.expect else None)]
    except (ValueError, KeyError, OSError) as e:
        print(f"armchaos: {e}", file=sys.stderr)
        return 2
    lead = a.armdoc_lead if a.armdoc else 0.0
    if a.armdoc and not ArmdocWatcher.available():
        print("armchaos: --armdoc needs armdoc (https://github.com/darshmenon/armdoc) on PYTHONPATH", file=sys.stderr)
        return 2
    msg = over_budget(steps, a.max_duration, lead)
    if msg:
        print(f"armchaos: {msg}", file=sys.stderr)
        return 2
    for s in steps:
        e = "" if s.expect is None else ("  expect: none" if s.expect == () else f"  expect: {', '.join(s.expect)}")
        print(f"plan: t={s.at:g}s for {s.dur:g}s  {s.label()}{e}")
    print(f"plan: relay {a.source} -> {a.out}" + (f", controllers via {a.cm}" if any(s.fault.controller for s in steps) else ""))
    print(f"plan: stops after {a.max_duration:g}s at most; controllers "
          + ("restored on Ctrl-C/SIGTERM" if a.restore_on_exit else "NOT restored on Ctrl-C/SIGTERM"))
    if a.dry_run:
        return 0
    if not a.armdoc and any(s.expect is not None for s in steps):
        print("note: `expect:` is only checked with --armdoc", file=sys.stderr)
    from armchaos.relay import run
    watcher = ArmdocWatcher(a.out) if a.armdoc else None
    res = run(steps, a.source, a.out, a.cm, a.events, lead=lead, max_duration=a.max_duration,
              restore_on_exit=a.restore_on_exit, watcher=watcher)
    results = []
    if watcher:
        if not watcher.alive() and not watcher.reports:
            print("armchaos: the armdoc probe produced no reports: " + " | ".join(watcher.other), file=sys.stderr)
        results = build_results(steps, res.windows, watcher.reports)
        print("\n" + format_table(results))
        ok = sum(r.status == "PASS" for r in results)
        bad = sum(r.status == "FAIL" for r in results)
        print(f"\n{ok} passed, {bad} failed, {sum(r.status == '-' for r in results)} unchecked, "
              f"{sum(r.status == 'SKIP' for r in results)} not run")
    else:
        print(f"\ndone: {len(res.windows)} of {len(steps)} fault(s) applied")
    return exit_code(results, res.unrestored, res.stopped_by if res.stopped_by in ("SIGINT", "SIGTERM") else None)


if __name__ == "__main__":
    sys.exit(main())
