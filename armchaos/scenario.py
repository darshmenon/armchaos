"""ROS-free scheduling: which faults are active when, and how a sample flows through them."""
from dataclasses import dataclass, field
from typing import List

from armchaos.faults import Fault, Sample, make_fault


@dataclass
class Step:
    at: float
    dur: float
    fault: Fault
    active: bool = False

    @property
    def end(self):
        return self.at + self.dur

    def label(self):
        p = ", ".join(f"{k}={v}" for k, v in self.fault.p.items() if v not in (None, ""))
        return f"{self.fault.name}({p})" if p else self.fault.name


def parse_scenario(data) -> List[Step]:
    """data: {'faults': [{'at': 5, 'for': 8, 'fault': 'freeze', ...params}]} -> sorted steps."""
    if not isinstance(data, dict) or not isinstance(data.get("faults"), list) or not data["faults"]:
        raise ValueError("scenario needs a non-empty 'faults:' list")
    steps = []
    for i, e in enumerate(data["faults"]):
        e = dict(e)
        if not all(isinstance(k, str) for k in e):
            raise ValueError(f"fault #{i + 1}: parameter names must be strings (YAML reads on/off/yes/no as booleans)")
        for k in ("at", "for", "fault"):
            if k not in e:
                raise ValueError(f"fault #{i + 1} is missing '{k}'")
        at, dur, name = float(e.pop("at")), float(e.pop("for")), e.pop("fault")
        if at < 0 or dur <= 0:
            raise ValueError(f"fault #{i + 1}: 'at' must be >= 0 and 'for' > 0")
        steps.append(Step(at, dur, make_fault(name, **e)))
    return sorted(steps, key=lambda s: s.at)


class Engine:
    def __init__(self, steps: List[Step]):
        self.steps = steps

    def end(self):
        return max(s.end for s in self.steps)

    def update(self, t):
        """Advance to scenario time t; returns (started, stopped) steps."""
        started, stopped = [], []
        for s in self.steps:
            on = s.at <= t < s.end
            if on and not s.active:
                s.fault.reset()
                s.active = True
                started.append(s)
            elif not on and s.active:
                s.active = False
                stopped.append(s)
        return started, stopped

    def _stream(self):
        return [s.fault for s in self.steps if s.active and not s.fault.controller]

    def process(self, now, sample: Sample) -> List[Sample]:
        return self._pipe(now, [sample], 0)

    def tick(self, now) -> List[Sample]:
        out = []
        chain = self._stream()
        for i, f in enumerate(chain):
            out += self._pipe(now, f.tick(now), i + 1)
        return out

    def _pipe(self, now, samples, start):
        chain = self._stream()
        for f in chain[start:]:
            nxt = []
            for s in samples:
                nxt += f.process(now, s)
            samples = nxt
        return samples
