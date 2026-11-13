"""The C++ stream faults and engine (armchaos_cpp) must shape a joint-state stream exactly like armchaos/faults.py."""
import math
import os
import random
import shutil
import statistics
import subprocess

import pytest

from armchaos.faults import Sample, make_fault
from armchaos.scenario import Engine, Step

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "armchaos_cpp")
NAMES = ["j0", "j1", "j2"]
pytestmark = pytest.mark.skipif(shutil.which("g++") is None, reason="needs g++")


@pytest.fixture(scope="module")
def cli(tmp_path_factory):
    out = str(tmp_path_factory.mktemp("cpp") / "armchaos_core_cli")
    subprocess.run(["g++", "-std=c++17", "-O2", "-I", os.path.join(ROOT, "include"),
                    os.path.join(ROOT, "src", "core_cli.cpp"), "-o", out], check=True)
    return out


def _f(x):
    return "nan" if math.isnan(x) else repr(float(x))


def build(faults, seed, hz=50, secs=14.0, with_vel=True, with_eff=True):
    """Event lines for the C++ cli plus the expected per-event outputs from Python."""
    rnd = random.Random(seed)
    steps = sorted((Step(at, dur, make_fault(name, **kw)) for at, dur, name, kw in faults), key=lambda s: s.at)
    eng = Engine(steps)
    lines = [f"F {at} {dur} {name} " + " ".join(f"{k}={v}" for k, v in kw.items()) for at, dur, name, kw in faults]
    py = []
    t = 0.0
    while t < secs:
        lines.append(f"U {t!r}"); eng.update(t); py.append([])
        pos = [rnd.uniform(-1, 1) for _ in NAMES]
        vel = [rnd.uniform(-1, 1) for _ in NAMES] if with_vel else []
        eff = [rnd.uniform(-5, 5) for _ in NAMES] if with_eff else []
        lines.append(f"S {t!r} 3 " + " ".join(
            f"{n} {_f(pos[i])} {_f(vel[i]) if vel else '-'} {_f(eff[i]) if eff else '-'}" for i, n in enumerate(NAMES)))
        try:
            out = eng.process(t, Sample(list(NAMES), pos, vel, eff))
        except KeyError:
            py.append("ERR"); out = None
        else:
            py.append(out)
        lines.append(f"T {t + 0.005!r}"); py.append(eng.tick(t + 0.005))
        t += 1.0 / hz
    return "\n".join(lines) + "\n", py


def run_cli(cli, text, tmp_path):
    p = tmp_path / "s.txt"
    p.write_text(text)
    out = subprocess.run([cli, str(p)], check=True, capture_output=True, text=True).stdout
    groups, cur = [], []
    for ln in out.splitlines():
        if ln == "--":
            groups.append(cur); cur = []
        else:
            cur.append(ln)
    return groups


def parse_o(line):
    tok = line.split()
    n = int(tok[1])
    names, pos, vel, eff = [], [], [], []
    for i in range(n):
        names.append(tok[2 + 4 * i]); pos.append(float(tok[3 + 4 * i]))
        v, e = tok[4 + 4 * i], tok[5 + 4 * i]
        if v != "-": vel.append(float(v))
        if e != "-": eff.append(float(e))
    return names, pos, vel, eff


def same(a, b):
    return len(a) == len(b) and all(x == y or (math.isnan(x) and math.isnan(y)) for x, y in zip(a, b))


CASES = {
    "freeze": [(2, 4, "freeze", {})],
    "slow": [(1, 8, "slow", {"hz": 2.0})],
    "stutter": [(1, 11, "stutter", {"up": 0.5, "down": 1.0})],
    "lag": [(1, 6, "lag", {"delay": 0.7})],
    "nan": [(1, 5, "nan", {"joint": "j1"})],
    "nan_default_joint": [(1, 5, "nan", {})],
    "spoof": [(1, 5, "spoof", {"joint": "j2", "offset": 7.0})],
    "squeeze": [(1, 5, "squeeze", {"joint": "j0", "effort": 80.0})],
    "spike": [(1, 5, "spike", {"joint": "j1", "velocity": 9.0})],
    "overlap": [(1, 8, "spoof", {"joint": "j0", "offset": 1.0}), (3, 4, "lag", {"delay": 0.4}), (5, 3, "nan", {"joint": "j2"})],
    "back_to_back": [(1, 3, "stutter", {"up": 0.3, "down": 0.3}), (4, 3, "freeze", {}), (8, 3, "stutter", {"up": 0.2, "down": 0.5})],
}


@pytest.mark.parametrize("name", sorted(CASES))
@pytest.mark.parametrize("seed", [1, 2])
def test_same_output_as_python(cli, tmp_path, name, seed):
    text, py = build(CASES[name], seed)
    cpp = run_cli(cli, text, tmp_path)
    assert len(cpp) == len(py)
    for i, (p, c) in enumerate(zip(py, cpp)):
        if p == "ERR":
            assert any(x.startswith("ERR") for x in c), (name, i)
            continue
        assert len(p) == len([x for x in c if x.startswith("O")]), f"{name} seed {seed} event {i}: {len(p)} vs {c}"
        for s, line in zip(p, [x for x in c if x.startswith("O")]):
            n, pos, vel, eff = parse_o(line)
            assert n == s.names and same(pos, s.pos) and same(vel, s.vel) and same(eff, s.eff), (name, seed, i)


def test_missing_joint_is_an_error_like_python(cli, tmp_path):
    case = [(1, 2, "nan", {"joint": "no_such_joint"})]
    text, py = build(case, 1, secs=4)
    cpp = run_cli(cli, text, tmp_path)
    assert "ERR" in py and any(any(x.startswith("ERR") for x in c) for c in cpp)


def test_scenario_covers_every_ported_fault():
    assert {c[0][2] for c in CASES.values()} >= {"freeze", "slow", "stutter", "lag", "nan", "spoof", "squeeze", "spike"}


def test_noise_is_deterministic_per_seed_and_has_the_right_spread(cli, tmp_path):
    def noise(seed, joint=None):
        kw = {"sigma": 0.05, "seed": seed}
        if joint: kw["joint"] = joint
        text, _ = build([(0, 13, "noise", kw)], 5, with_vel=False, with_eff=False)
        rows = [parse_o(x) for g in run_cli(cli, text, tmp_path) for x in g if x.startswith("O")]
        return rows, text
    a, text = noise(0)
    b, _ = noise(0)
    c, _ = noise(1)
    assert a == b and a != c
    # compare with the clean samples from the scenario text
    ins = [[float(ln.split()[4 + 4 * i]) for i in range(3)] for ln in text.splitlines() if ln.startswith("S")]
    diffs = [r[1][i] - ins[k][i] for k, r in enumerate(a) for i in range(3)]
    assert abs(statistics.mean(diffs)) < 0.01 and 0.04 < statistics.pstdev(diffs) < 0.06
    one, _ = noise(0, joint="j1")
    own = [[r[1][i] - ins[k][i] for k, r in enumerate(one)] for i in range(3)]
    assert max(map(abs, own[0])) == 0 and max(map(abs, own[2])) == 0 and max(map(abs, own[1])) > 0


def test_unknown_fault_and_parameter_are_rejected(cli, tmp_path):
    for line in ("F 0 1 bogus", "F 0 1 freeze hz=2", "F 0 1 slow nope=1", "F -1 1 freeze", "F 0 0 freeze"):
        p = tmp_path / "bad.txt"
        p.write_text(line + "\n")
        out = subprocess.run([cli, str(p)], capture_output=True, text=True).stdout
        assert "ERR" in out, line
