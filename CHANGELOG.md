# Changelog

## Unreleased
- New `armchaos_cpp` colcon package (in this repo): `ros2 run armchaos_cpp armchaos_fast` is a C++ relay with the nine stream faults (`freeze`, `slow`, `stutter`, `lag`, `noise`, `nan`, `spoof`, `squeeze`, `spike`) and the same scheduling as the Python relay. On a 500 Hz stream (one 15 s run each, busy machine) it used 4.6% of a core and 22 MB against 18.2% and 60 MB for the Python relay; median relay latency 235 us against 326 us. `deactivate` (controller_manager calls and the restore-on-exit safety) stays Python only.
- `test/test_cpp_equivalence.py` replays identical event sequences (11 fault setups including overlapping and back-to-back faults, two seeds each) through the Python `Engine` and the C++ engine and requires bit-identical samples; `noise` is checked for determinism per seed and for the right spread, since the two languages use different random generators.
- `test/cpp_relay_check.py` and a CI step run the C++ relay against `fake_cm.py` and require the stream to stop during a freeze and resume after it.

## 0.2.0
- Safe shutdown: Ctrl-C and SIGTERM stop the relay and switch every deactivated controller back on (blocking, with retries) before exiting; exit code 130 / 143, or 3 if a controller could not be restored. `--restore-on-exit` (default) / `--no-restore-on-exit`.
- `--max-duration SECONDS` (default 600): a longer scenario is refused, and a run that overshoots it is stopped and restored.
- `expect:` per scenario step and `--expect CODE` for `inject`; `--armdoc` runs armdoc on the faulted stream and prints a pass/fail table, exiting 1 if an expected code did not fire (`--armdoc-lead` sets the healthy baseline, default 4 s).
- `armchaos --version`, `pyproject.toml` so `pip install git+https://github.com/darshmenon/armchaos` works (colcon still builds it).
- CI workflow: ROS-free unit tests and a pip install check, plus a ROS Humble job that runs the restore check and an armdoc end-to-end scenario.
- New tests: `test/test_report.py` (ROS-free), `test/restore_check.py` (against `test/fake_cm.py`), `test/scenario_expect.yaml`.

## 0.1.0
- First version: break a ros2_control arm on purpose, then restore it.
- Stream faults (armchaos relays `/joint_states` to a second topic through the active faults): `freeze`, `slow`, `stutter`, `lag`, `noise`, `nan`, `spoof`, `squeeze`, `spike`.
- Controller fault: `deactivate` a controller through `controller_manager/switch_controller`, reactivated when the fault ends (and on Ctrl-C).
- `armchaos run scenario.yaml`, `armchaos inject FAULT key=value --for N`, `armchaos list`, `--dry-run`, `--events` (JSON lines).
- Tested against armdoc on a fake ros2_control graph and on a UR3 in Gazebo; see the README for which faults armdoc names and which it misses.
