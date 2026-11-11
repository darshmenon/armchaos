# armchaos

**Does your arm stack notice when something breaks?** Break a ros2_control arm on purpose (frozen, late, noisy, NaN or spoofed joint states, a deactivated controller), then restore it. Pair it with [armdoc](https://github.com/darshmenon/armdoc) to check the doctor names each fault.

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
  - {at: 4,  for: 8, fault: freeze}
  - {at: 20, for: 8, fault: spoof, joint: shoulder_pan_joint, offset: 7.0}
  - {at: 40, for: 10, fault: deactivate, controller: arm_controller}
```

## How it works

Stream faults: armchaos subscribes to `--source` (default `/joint_states`), passes each sample through the active faults, and publishes the result on `--out` (default `/joint_states_chaos`). **The real `/joint_states` is never touched**, so point the thing under test at `--out` (a topic remap is enough), or move your real source to another topic and relay it back onto `/joint_states`.

Controller fault: `deactivate` calls `controller_manager/switch_controller` on the real controller manager and reactivates the controller when the fault ends (and on Ctrl-C). Unlike the stream faults this one does change the running system.

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
- **Not tested:** ROS 2 Jazzy, MoveIt, a real robot, Ctrl-C restoring a deactivated controller, two arms at once. `--dry-run` was only run by hand.

## Reproduce

```bash
python3 test/fake_arm_raw.py &                     # or a simulator publishing /joint_states_raw
python3 test/e2e.py test/scenario_stream.yaml --source /joint_states_raw
```

`test/e2e.py` starts armchaos and a remapped armdoc, then prints the table above. For a simulator, use `--source /joint_states` and `test/scenario_gazebo.yaml` (UR3 joint names).

## Limits

- Joint-state faults only, plus one controller fault. No TF, camera or force-sensor faults yet.
- Stream faults apply to the relayed topic, so they need a remap in the stack under test.
- YAML reads `on`, `off`, `yes` and `no` as booleans, so `stutter` uses `up` and `down`; a boolean key gets a clear error.
- Needs `controller_manager_msgs` and PyYAML.
