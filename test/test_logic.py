import math

import pytest

from armchaos.faults import Sample, make_fault
from armchaos.scenario import Engine, Step, parse_scenario


def sample(v=0.0):
    return Sample(["j0", "j1"], [0.1 + v, 0.2], [0.0, 0.0], [1.0, 1.0])


def test_freeze_drops_everything():
    assert make_fault("freeze").process(0.0, sample()) == []


def test_slow_keeps_one_per_period():
    f = make_fault("slow", hz=1.0)
    kept = [t for t in [i * 0.1 for i in range(25)] if f.process(t, sample())]
    assert kept == [0.0, 1.0, 2.0]


def test_stutter_has_gaps():
    f = make_fault("stutter", up=1.0, down=2.0)
    on = [f.process(t, sample()) != [] for t in (0.0, 0.5, 1.5, 2.5, 3.1, 3.5)]
    assert on == [True, True, False, False, True, True]


def test_lag_delays_and_keeps_order():
    f = make_fault("lag", delay=1.0)
    assert f.process(0.0, sample(0.0)) == [] and f.process(0.5, sample(0.5)) == []
    assert f.tick(0.9) == []
    assert [s.pos[0] for s in f.tick(1.2)] == [0.1]
    assert [s.pos[0] for s in f.tick(2.0)] == [0.6]


def test_noise_is_reproducible_and_does_not_mutate_input():
    s = sample()
    a = make_fault("noise", sigma=0.1, seed=3).process(0, s)[0].pos
    b = make_fault("noise", sigma=0.1, seed=3).process(0, s)[0].pos
    assert a == b and a != s.pos and s.pos == [0.1, 0.2]


def test_nan_spoof_squeeze_spike_target_one_joint():
    s = sample()
    assert math.isnan(make_fault("nan", joint="j1").process(0, s)[0].pos[1])
    assert make_fault("spoof", joint="j0", offset=5).process(0, s)[0].pos == [5.1, 0.2]
    assert make_fault("squeeze", joint="j1", effort=99).process(0, s)[0].eff == [1.0, 99]
    assert make_fault("spike", joint="j0", velocity=7).process(0, s)[0].vel == [7, 0.0]
    assert s.pos == [0.1, 0.2] and s.eff == [1.0, 1.0]


def test_default_joint_is_first_and_unknown_joint_errors():
    assert math.isnan(make_fault("nan").process(0, sample())[0].pos[0])
    with pytest.raises(KeyError):
        make_fault("nan", joint="nope").process(0, sample())


def test_unknown_fault_and_param_rejected():
    with pytest.raises(ValueError, match="unknown fault"):
        make_fault("explode")
    with pytest.raises(ValueError, match="no parameter"):
        make_fault("freeze", hz=1)


def test_scenario_parse_and_validation():
    steps = parse_scenario({"faults": [{"at": 9, "for": 2, "fault": "freeze"},
                                       {"at": 1, "for": 3, "fault": "nan", "joint": "j1"}]})
    assert [s.fault.name for s in steps] == ["nan", "freeze"]        # sorted by start time
    for bad in ({}, {"faults": []}, {"faults": [{"at": 1, "for": 2}]},
                {"faults": [{"at": 1, "for": 0, "fault": "freeze"}]}):
        with pytest.raises(ValueError):
            parse_scenario(bad)


def test_engine_applies_then_restores():
    e = Engine([Step(2.0, 3.0, make_fault("spoof", offset=1.0))])
    assert e.process(1.0, sample())[0].pos[0] == pytest.approx(0.1)
    started, _ = e.update(2.0)
    assert len(started) == 1 and e.process(2.5, sample())[0].pos[0] == pytest.approx(1.1)
    _, stopped = e.update(5.0)
    assert len(stopped) == 1 and e.process(5.5, sample())[0].pos[0] == pytest.approx(0.1)
    assert e.end() == 5.0


def test_engine_chains_faults_in_order():
    e = Engine([Step(0, 10, make_fault("lag", delay=1.0)), Step(0, 10, make_fault("spoof", offset=1.0))])
    e.update(0.0)
    assert e.process(0.0, sample()) == []
    out = e.tick(1.1)                       # delayed sample still passes through the spoof
    assert len(out) == 1 and out[0].pos[0] == pytest.approx(1.1)


def test_controller_fault_does_not_touch_the_stream():
    e = Engine([Step(0, 5, make_fault("deactivate", controller="arm_controller"))])
    e.update(0.0)
    assert e.process(1.0, sample())[0].pos == [0.1, 0.2]


def test_yaml_boolean_keys_get_a_clear_error():
    with pytest.raises(ValueError, match="parameter names must be strings"):
        parse_scenario({"faults": [{"at": 1, "for": 2, "fault": "stutter", True: 1.0}]})
