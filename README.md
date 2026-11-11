# armchaos

**Does your arm stack notice when something breaks?** Break a ros2_control arm on purpose (frozen, late, noisy, NaN or spoofed joint states, a deactivated controller), then restore it. Pair it with [armdoc](https://github.com/darshmenon/armdoc) to check the doctor names each fault.

## Quick start

```bash
# install (ROS 2 Humble sourced; use --system-site-packages if you use a venv)
pip install git+https://github.com/darshmenon/armchaos       # or: colcon build --packages-select armchaos

armchaos list                                    # the faults and their parameters
armchaos inject freeze --for 8 --dry-run         # show the plan, touch nothing
armchaos inject freeze --for 8                   # joint_states stop for 8 s (point your stack at /joint_states_chaos)
```

Try it without a robot, from a clone of this repo (a fake arm and controller manager, plus armdoc on `PYTHONPATH`):

```bash
python3 test/fake_cm.py &                        # publishes /joint_states_raw, serves list/switch_controller
armchaos run test/scenario_expect.yaml --armdoc --source /joint_states_raw
```

The last command ends with a pass/fail table and exits non-zero if an expected armdoc code did not fire, so it works as a CI test:

```
#  fault                                  when       expect                             armdoc said                        result
-  -------------------------------------  ---------  ---------------------------------  ---------------------------------  ------
1  freeze                                 t=2s +6s   JOINT_STATES_STALE                 JOINT_STATES_STALE                 PASS
2  nan(joint=j1)                          t=12s +5s  JOINT_NAN                          JOINT_NAN                          PASS
3  deactivate(controller=arm_controller)  t=21s +7s  CTRL_INACTIVE, NO_ACTIVE_ARM_CTRL  CTRL_INACTIVE, NO_ACTIVE_ARM_CTRL  PASS
4  noise(sigma=0.02, seed=0)              t=32s +5s  -                                  nothing                            -
```

(Real output against armdoc 0.2.0 and `test/fake_cm.py`: `docs/e2e_expect.txt`.)

## Commands

```bash
ros2 run armchaos armchaos list                                     # faults and their parameters
ros2 run armchaos armchaos inject freeze --for 8                    # joint_states stop for 8 s
ros2 run armchaos armchaos inject spoof joint=shoulder_pan_joint offset=7 --at 5 --for 6
ros2 run armchaos armchaos inject deactivate controller=arm_controller --for 10
ros2 run armchaos armchaos run scenario.yaml --events out.jsonl     # several faults on a timeline
ros2 run armchaos armchaos inject nan --dry-run                     # show the plan, touch nothing
```


```yaml
# scenario.yaml: each fault starts at `at` seconds and lasts `for` seconds
faults:
  - {at: 4,  for: 8, fault: freeze, expect: JOINT_STATES_STALE}
  - {at: 20, for: 8, fault: spoof, joint: shoulder_pan_joint, offset: 7.0, expect: LIMIT_POS}
  - {at: 40, for: 10, fault: deactivate, controller: arm_controller, expect: [CTRL_INACTIVE, NO_ACTIVE_ARM_CTRL]}
```

### Checking armdoc: `--armdoc` and `expect:`

`expect:` names the armdoc codes that must appear while a fault is active: one code, a list (all of them must appear), or `none` (armdoc must stay quiet). A step without `expect:` is only reported, not judged.

With `--armdoc`, armchaos starts armdoc (it must be importable: installed, or on `PYTHONPATH`) on the faulted stream, waits `--armdoc-lead` seconds (default 4) so armdoc sees a healthy baseline, then runs the scenario and compares each step's window, plus 1.5 s after it ends, with the codes armdoc printed. It runs armdoc live for the whole run (one report per second), not once per step, so that short and intermittent faults are not missed. `inject` takes `--expect CODE` (repeatable) the same way. Without `--armdoc`, `expect:` is ignored and armchaos says so.

Exit codes: `0` ok, `1` an `expect:` failed, `2` bad usage or a scenario longer than `--max-duration`, `3` a controller could not be restored, `130` / `143` stopped by Ctrl-C / SIGTERM.

## Safety

- **Controllers are always put back.** If a run stops early (Ctrl-C, SIGTERM, an error, or the guard below), armchaos reactivates every controller it switched off, blocking with retries, and exits `3` if it cannot (it prints which controller to switch on by hand). `--no-restore-on-exit` leaves them off instead. A fault that ends normally always reactivates its controller.
- **Nothing runs forever.** `--max-duration SECONDS` (default 600) refuses a scenario that is longer, and also stops the relay if the run overshoots it, restoring controllers first.
- Stream faults never touch the real `/joint_states`; only `deactivate` changes the running system.

## How it works

Stream faults: armchaos subscribes to `--source` (default `/joint_states`), passes each sample through the active faults, and publishes the result on `--out` (default `/joint_states_chaos`). **The real `/joint_states` is never touched**, so point the thing under test at `--out` (a topic remap is enough), or move your real source to another topic and relay it back onto `/joint_states`.

Controller fault: `deactivate` calls `controller_manager/switch_controller` on the real controller manager and reactivates the controller when the fault ends (and on Ctrl-C or SIGTERM, see Safety). Unlike the stream faults this one does change the running system.

## Faults, and what armdoc said

Real runs, one fault at a time with 6 s of recovery in between, `armdoc` live with a 1 s report period. "+2.2s" is how long after the fault started the first report named it. Raw output: `docs/e2e_*.txt` and `.json`.

| Fault | What it does | armdoc code | Fake arm | UR3 in Gazebo |
|---|---|---|---|---|
| `freeze` | joint_states stop | `JOINT_STATES_STALE` | +2.2 s | +2.2 s |
| `slow hz=0.2` | one sample every 5 s | `JOINT_STATES_STALE` | +3.0 s | +2.3 s |
| `stutter up=1 down=4` | 1 s on, 4 s silent, repeating | `JOINT_STATES_STALE` | +3.8 s | +3.4 s |
| `lag delay=3` | every sample 3 s late | `JOINT_STATES_STALE` | +2.4 s | +2.5 s |
| `noise sigma=0.02` | Gaussian noise on positions | **none** (armdoc has no noise check) | none | none |
| `nan` | one joint position is NaN | `JOINT_NAN` | +0.7 s | +0.5 s |
| `spoof offset=...` | one joint shifted past its limit | `LIMIT_POS` | +0.8 s | +0.5 s |
| `squeeze effort=100` | one joint reports effort above its limit | `EFFORT_SAT` | +2.9 s | +2.5 s |
| `spike velocity=10` | one joint reports a velocity above its limit | `LIMIT_VEL` | +0.0 s | +0.5 s |
| `deactivate` | arm controller switched off | `CTRL_INACTIVE`, `NO_ACTIVE_ARM_CTRL` | +2.0 s | +1.6 s |

Every fault was followed by an `OK` report within 6 s of it ending, on both setups.

Two honest notes on those rows:
- `lag` is reported as stale, not as late: the first `delay` seconds are silent, and after that the stream flows normally, only shifted. armdoc has no check for a constant delay.
- `noise` is a real gap in armdoc, not a bug in armchaos. Small position noise does not break a limit, so nothing fires.

## What was and wasn't tested

- **Fake arm** (`test/fake_arm_raw.py`, armdoc's fake graph on a remapped topic): all nine stream faults. `deactivate` ran against `test/fake_cm.py`, a small stateful fake controller manager, because the fake arm's controller list is fixed.
- **UR3 in Gazebo Harmonic** (ros2_control, `arm_controller` + `joint_state_broadcaster`, peg tool, no MoveIt): all nine stream faults plus `deactivate` on the real controller manager. The sim reported `arm_controller` active again after the run.
- Armdoc was pointed at the faulted stream with a ROS remap (`test/armdoc_remap.py`); the sim's real `/joint_states` stayed untouched.
- Unit tests (`pytest test`) cover the fault models and the scheduler without ROS.
- **Safety (0.2.0):** `test/restore_check.py` runs against `test/fake_cm.py`, a fake controller manager, and checks five cases: SIGINT and SIGTERM restore the controller and exit 130 / 143, the `--max-duration` guard stops a run and restores, `--no-restore-on-exit` leaves it off, and a normal run restores. All five pass. This was not run against the real `ros2_control` controller manager (a Gazebo run was out of scope), so the switching is only verified against the fake one.
- **`--armdoc` summary (0.2.0):** run on the fake graph with armdoc v0.2.0: three expectations passed. With a newer armdoc that has a noise check, a step with `expect: none` failed as it should (exit 1). `pip install .` was tested in a clean venv and `colcon build` still works.
- **Not tested:** ROS 2 Jazzy, MoveIt, a real robot, two arms at once, the GitHub Actions workflow itself (only its commands, locally).

## Reproduce

```bash
python3 test/fake_arm_raw.py &                     # or a simulator publishing /joint_states_raw
python3 test/e2e.py test/scenario_stream.yaml --source /joint_states_raw
```

`test/e2e.py` starts armchaos and a remapped armdoc, then prints the fault-to-code table above (for pass/fail checks use `armchaos run ... --armdoc` instead). For a simulator, use `--source /joint_states` and `test/scenario_gazebo.yaml` (UR3 joint names).

## Limits

- Joint-state faults only, plus one controller fault. No TF, camera or force-sensor faults yet.
- Stream faults apply to the relayed topic, so they need a remap in the stack under test.
- YAML reads `on`, `off`, `yes` and `no` as booleans, so `stutter` uses `up` and `down`; a boolean key gets a clear error.
- Needs `controller_manager_msgs` and PyYAML.
