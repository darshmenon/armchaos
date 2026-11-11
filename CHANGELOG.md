# Changelog

## 0.1.0
- First version: break a ros2_control arm on purpose, then restore it.
- Stream faults (armchaos relays `/joint_states` to a second topic through the active faults): `freeze`, `slow`, `stutter`, `lag`, `noise`, `nan`, `spoof`, `squeeze`, `spike`.
- Controller fault: `deactivate` a controller through `controller_manager/switch_controller`, reactivated when the fault ends (and on Ctrl-C).
- `armchaos run scenario.yaml`, `armchaos inject FAULT key=value --for N`, `armchaos list`, `--dry-run`, `--events` (JSON lines).
- Tested against armdoc on a fake ros2_control graph and on a UR3 in Gazebo; see the README for which faults armdoc names and which it misses.
