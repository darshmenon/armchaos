#!/usr/bin/env python3
"""Stateful fake controller manager + arm: list_controllers/switch_controller really change state.
Publishes /joint_states_raw and a latched /robot_description."""
import math
import time

import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

URDF = '<robot name="r"><link name="a"/><link name="b"/>' + "".join(
    f'<joint name="j{i}" type="revolute"><parent link="a"/><child link="b"/>'
    f'<limit lower="-3" upper="3" velocity="3" effort="50"/></joint>' for i in range(3)) + "</robot>"


def main():
    rclpy.init()
    n = Node("fake_cm")
    t0 = time.monotonic()
    state = {"arm_controller": "active"}
    js = n.create_publisher(JointState, "/joint_states_raw", 10)
    n.create_publisher(String, "/robot_description", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)).publish(String(data=URDF))

    def lst(req, res):
        res.controller = [ControllerState(name=k, type="joint_trajectory_controller/JointTrajectoryController", state=v)
                          for k, v in state.items()]
        return res

    def sw(req, res):
        for c in req.deactivate_controllers:
            state[c] = "inactive"
        for c in req.activate_controllers:
            state[c] = "active"
        res.ok = True
        return res
    n.create_service(ListControllers, "/controller_manager/list_controllers", lst)
    n.create_service(SwitchController, "/controller_manager/switch_controller", sw)

    def tick():
        t = time.monotonic() - t0
        m = JointState(name=["j0", "j1", "j2"], position=[math.sin(t + i) for i in range(3)],
                       velocity=[0.0] * 3, effort=[0.0] * 3)
        m.header.stamp = n.get_clock().now().to_msg()
        js.publish(m)
    n.create_timer(0.02, tick)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass


if __name__ == "__main__":
    main()
