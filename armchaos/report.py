"""ROS-free end-of-run summary: which armdoc codes fired during each fault, and did they match `expect:`."""
from dataclasses import dataclass, field
from typing import List, Optional, Set, Tuple


@dataclass
class StepResult:
    label: str
    at: float
    dur: float
    expect: Optional[Tuple[str, ...]]
    ran: bool = True
    seen: Set[str] = field(default_factory=set)
    status: str = "-"           # PASS, FAIL, - (nothing to check), SKIP (never started)


def judge(expect, seen, ran=True):
    if not ran:
        return "SKIP"
    if expect is None:
        return "-"
    if expect == ():
        return "PASS" if not seen else "FAIL"
    return "PASS" if set(expect) <= set(seen) else "FAIL"


def codes_in_window(reports, start, stop, tail=1.5):
    """reports: [(wall_time, [codes])]. Codes armdoc printed between the fault starting and `tail` s after it ended."""
    seen = set()
    for t, codes in reports:
        if start <= t <= stop + tail:
            seen.update(codes)
    return seen


def build_results(steps, windows, reports, tail=1.5):
    """windows: {id(step): (wall_start, wall_stop)} for the steps that ran."""
    out = []
    for s in steps:
        w = windows.get(id(s))
        r = StepResult(s.label(), s.at, s.dur, s.expect, ran=w is not None)
        if w is not None:
            r.seen = codes_in_window(reports, w[0], w[1] if w[1] is not None else w[0], tail)
        r.status = judge(s.expect, r.seen, r.ran)
        out.append(r)
    return out


def format_table(results: List[StepResult]) -> str:
    def exp(r):
        return "-" if r.expect is None else ("none" if r.expect == () else ", ".join(r.expect))
    rows = [("#", "fault", "when", "expect", "armdoc said", "result")]
    for i, r in enumerate(results, 1):
        rows.append((str(i), r.label, f"t={r.at:g}s +{r.dur:g}s", exp(r),
                     ", ".join(sorted(r.seen)) or ("(not run)" if not r.ran else "nothing"), r.status))
    w = [max(len(row[c]) for row in rows) for c in range(6)]
    lines = ["  ".join(cell.ljust(w[c]) for c, cell in enumerate(row)).rstrip() for row in rows]
    lines.insert(1, "  ".join("-" * x for x in w))
    return "\n".join(lines)


def exit_code(results, unrestored=(), interrupted_by=None):
    """0 all checks passed, 1 an expectation failed, 3 a controller could not be restored, 130/143 interrupted."""
    if unrestored:
        return 3
    if interrupted_by == "SIGINT":
        return 130
    if interrupted_by == "SIGTERM":
        return 143
    if any(r.status == "FAIL" for r in results):
        return 1
    return 0
