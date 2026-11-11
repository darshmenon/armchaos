"""ROS-free fault models: each one reshapes a stream of joint-state samples the way a real failure would."""
import math
import random
from collections import deque
from dataclasses import dataclass, replace
from typing import List, Optional


@dataclass
class Sample:
    names: list
    pos: list
    vel: list
    eff: list
    stamp: object = None


class Fault:
    """Base: `process` is called per incoming sample, `tick` at ~100 Hz for delayed output."""
    name = ""
    help = ""
    controller = False       # True: acts on the controller manager instead of the joint-state stream

    def __init__(self, **kw):
        self.p = {**self.defaults(), **kw}
        self.reset()

    def defaults(self):
        return {}

    def reset(self):
        pass

    def process(self, now, s: Sample) -> List[Sample]:
        return [s]

    def tick(self, now) -> List[Sample]:
        return []

    def _idx(self, s):
        j = self.p.get("joint")
        if j in (None, ""):
            return 0
        if j not in s.names:
            raise KeyError(f"joint {j!r} not in {s.names}")
        return s.names.index(j)


class Freeze(Fault):
    name, help = "freeze", "joint_states stop completely (dead broadcaster or controller manager)"

    def process(self, now, s):
        return []


class Slow(Fault):
    name, help = "slow", "joint_states drop to a low rate (hz)"

    def defaults(self):
        return {"hz": 0.2}

    def reset(self):
        self.last = None

    def process(self, now, s):
        if self.last is None or now - self.last >= 1.0 / self.p["hz"]:
            self.last = now
            return [s]
        return []


class Stutter(Fault):
    name, help = "stutter", "joint_states flow for `up` s, then go silent for `down` s, repeating"

    def defaults(self):
        return {"up": 1.0, "down": 4.0}

    def reset(self):
        self.t0 = None

    def process(self, now, s):
        if self.t0 is None:
            self.t0 = now
        return [s] if (now - self.t0) % (self.p["up"] + self.p["down"]) < self.p["up"] else []


class Lag(Fault):
    name, help = "lag", "joint_states arrive `delay` s late (every sample, order kept)"

    def defaults(self):
        return {"delay": 3.0}

    def reset(self):
        self.q = deque()

    def process(self, now, s):
        self.q.append((now + self.p["delay"], s))
        return []

    def tick(self, now):
        out = []
        while self.q and self.q[0][0] <= now:
            out.append(self.q.popleft()[1])
        return out


class Noise(Fault):
    name, help = "noise", "Gaussian noise (sigma rad) on joint positions"

    def defaults(self):
        return {"sigma": 0.02, "joint": None, "seed": 0}

    def reset(self):
        self.rng = random.Random(self.p["seed"])

    def process(self, now, s):
        pos = list(s.pos)
        for i in range(len(pos)):
            if self.p["joint"] in (None, "") or s.names[i] == self.p["joint"]:
                pos[i] += self.rng.gauss(0.0, self.p["sigma"])
        return [replace(s, pos=pos)]


class Nan(Fault):
    name, help = "nan", "one joint position becomes NaN (diverged controller or driver)"

    def defaults(self):
        return {"joint": None}

    def process(self, now, s):
        pos = list(s.pos)
        pos[self._idx(s)] = math.nan
        return [replace(s, pos=pos)]


class Spoof(Fault):
    name, help = "spoof", "one joint position is shifted by `offset` rad (e.g. past its limit)"

    def defaults(self):
        return {"joint": None, "offset": 1.0}

    def process(self, now, s):
        pos = list(s.pos)
        pos[self._idx(s)] += self.p["offset"]
        return [replace(s, pos=pos)]


class Squeeze(Fault):
    name, help = "squeeze", "one joint reports a fixed effort (N*m), e.g. pinned against contact"

    def defaults(self):
        return {"joint": None, "effort": 100.0}

    def process(self, now, s):
        eff = list(s.eff) if s.eff else [0.0] * len(s.pos)
        eff[self._idx(s)] = self.p["effort"]
        return [replace(s, eff=eff)]


class Spike(Fault):
    name, help = "spike", "one joint reports a fixed velocity (rad/s) above its limit"

    def defaults(self):
        return {"joint": None, "velocity": 10.0}

    def process(self, now, s):
        vel = list(s.vel) if s.vel else [0.0] * len(s.pos)
        vel[self._idx(s)] = self.p["velocity"]
        return [replace(s, vel=vel)]


class Deactivate(Fault):
    name, help = "deactivate", "deactivate a controller through controller_manager, reactivate it afterwards"
    controller = True

    def defaults(self):
        return {"controller": "arm_controller"}


FAULTS = {c.name: c for c in (Freeze, Slow, Stutter, Lag, Noise, Nan, Spoof, Squeeze, Spike, Deactivate)}


def make_fault(name, **params) -> Fault:
    if name not in FAULTS:
        raise ValueError(f"unknown fault {name!r}; choose from {', '.join(sorted(FAULTS))}")
    cls = FAULTS[name]
    bad = set(params) - set(cls().defaults())
    if bad:
        raise ValueError(f"fault {name!r} has no parameter(s) {', '.join(sorted(bad))}; "
                         f"valid: {', '.join(cls().defaults()) or 'none'}")
    return cls(**params)
