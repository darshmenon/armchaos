#!/usr/bin/env python3
"""Run armdoc's fake_arm.py with /joint_states moved to /joint_states_raw, so armchaos can sit in between."""
import os
import runpy
import sys

import rclpy

path = os.environ.get("FAKE_ARM", os.path.expanduser("~/armdoc/test/fake_arm.py"))
_init = rclpy.init
rclpy.init = lambda **k: _init(args=["--ros-args", "-r", "/joint_states:=/joint_states_raw"])
sys.argv = [path] + sys.argv[1:]
runpy.run_path(path, run_name="__main__")
