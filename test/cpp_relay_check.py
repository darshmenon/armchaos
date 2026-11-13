#!/usr/bin/env python3
"""Live check of the C++ relay: run next to test/fake_cm.py (publishes /joint_states_raw), with
   ros2 run armchaos_cpp armchaos_fast --ros-args -p "faults:=['3,3,freeze']" -p linger:=2.0
Counts /joint_states_chaos messages per second and requires: flowing before the fault, silent inside it, flowing after."""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

rclpy.init()
n = Node("cpp_relay_check")
buckets = {}
t_first = [None]


def cb(_):
    now = time.monotonic()
    t_first[0] = t_first[0] or now
    k = int(now - t_first[0])
    buckets[k] = buckets.get(k, 0) + 1


n.create_subscription(JointState, "/joint_states_chaos", cb, qos_profile_sensor_data)
end = time.monotonic() + 11
while time.monotonic() < end and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.05)
counts = [buckets.get(i, 0) for i in range(10)]
print("messages per second on /joint_states_chaos:", counts)
before, after = counts[:2], counts[-2:]
# the fault covers relay seconds 3-6; the first message may arrive a little after the relay starts, so check the middle
ok = min(before) > 5 and min(after) > 5 and 0 in counts[3:7] and counts.count(0) >= 2
print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
