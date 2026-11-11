import pytest

from armchaos import cli
from armchaos.probe import ArmdocWatcher, parse_report_line
from armchaos.report import build_results, codes_in_window, exit_code, format_table, judge
from armchaos.scenario import Step, over_budget, parse_expect, parse_scenario
from armchaos.faults import make_fault


def step(name="freeze", at=2.0, dur=6.0, expect=None, **kw):
    return Step(at, dur, make_fault(name, **kw), expect=expect)


def test_expect_forms():
    assert parse_expect("JOINT_NAN") == ("JOINT_NAN",)
    assert parse_expect(["A_B", "C"]) == ("A_B", "C")
    assert parse_expect("none") == () and parse_expect([]) == ()
    for bad in (5, None, ["lower_case"], ["has space"], [1]):
        with pytest.raises(ValueError):
            parse_expect(bad)


def test_scenario_reads_expect_and_leaves_params_alone():
    steps = parse_scenario({"faults": [
        {"at": 1, "for": 2, "fault": "nan", "joint": "j1", "expect": "JOINT_NAN"},
        {"at": 5, "for": 2, "fault": "freeze"}]})
    assert steps[0].expect == ("JOINT_NAN",) and steps[0].fault.p["joint"] == "j1"
    assert steps[1].expect is None


def test_judge():
    assert judge(None, {"X"}) == "-"
    assert judge(("A",), {"A", "B"}) == "PASS"
    assert judge(("A", "B"), {"A"}) == "FAIL"            # every listed code must fire
    assert judge((), set()) == "PASS" and judge((), {"A"}) == "FAIL"
    assert judge(("A",), set(), ran=False) == "SKIP"


def test_codes_are_taken_from_the_fault_window_plus_a_short_tail():
    reports = [(1.0, ["OLD"]), (5.0, ["A"]), (9.0, ["B"]), (9.4, ["C"]), (20.0, ["LATE"])]
    assert codes_in_window(reports, 4.0, 9.0, tail=1.0) == {"A", "B", "C"}


def test_results_table_and_exit_codes():
    steps = [step(expect=("JOINT_STATES_STALE",)), step("nan", at=10, expect=("JOINT_NAN",)), step("spoof", at=20)]
    windows = {id(steps[0]): (100.0, 106.0), id(steps[1]): (110.0, 115.0)}      # third step never started
    reports = [(103.0, ["JOINT_STATES_STALE"]), (112.0, [])]
    res = build_results(steps, windows, reports)
    assert [r.status for r in res] == ["PASS", "FAIL", "SKIP"]
    table = format_table(res)
    assert "JOINT_STATES_STALE" in table and "FAIL" in table and "(not run)" in table
    assert exit_code(res) == 1
    assert exit_code(res[:1]) == 0
    assert exit_code(res[:1], unrestored=["arm_controller"]) == 3        # restore failures outrank everything
    assert exit_code(res[:1], interrupted_by="SIGINT") == 130 and exit_code(res[:1], interrupted_by="SIGTERM") == 143


def test_budget_guard_refuses_long_scenarios():
    steps = [step(at=5, dur=30)]
    assert over_budget(steps, 600) is None
    assert "--max-duration" in over_budget(steps, 30)
    assert over_budget(steps, 39, lead=4.0) is None and over_budget(steps, 39, lead=4.5)


def test_parse_report_line():
    assert parse_report_line('[{"level": "ERROR", "code": "JOINT_NAN", "msg": "x"}]') == ["JOINT_NAN"]
    assert parse_report_line("[]") == []
    for junk in ("Traceback (most recent call last):", "", "[not json", '[{"level": 1}]'):
        assert parse_report_line(junk) is None


def test_cli_dry_run_prints_the_plan_and_guards(capsys):
    assert cli.main(["inject", "freeze", "--dry-run", "--expect", "JOINT_STATES_STALE"]) == 0
    out = capsys.readouterr().out
    assert "expect: JOINT_STATES_STALE" in out and "restored on Ctrl-C/SIGTERM" in out
    assert cli.main(["inject", "freeze", "--dry-run", "--no-restore-on-exit"]) == 0
    assert "NOT restored" in capsys.readouterr().out


def test_cli_refuses_a_run_longer_than_max_duration(capsys):
    assert cli.main(["inject", "freeze", "--for", "100", "--max-duration", "30", "--dry-run"]) == 2
    assert "--max-duration" in capsys.readouterr().err


def test_cli_armdoc_flag_needs_armdoc(monkeypatch, capsys):
    monkeypatch.setattr(ArmdocWatcher, "available", staticmethod(lambda: False))
    assert cli.main(["inject", "freeze", "--armdoc", "--dry-run"]) == 2
    assert "needs armdoc" in capsys.readouterr().err
