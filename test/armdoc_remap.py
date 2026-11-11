#!/usr/bin/env python3
"""Run armdoc with /joint_states remapped to the faulted stream (default /joint_states_chaos).
Usage: armdoc_remap.py [armdoc args...]; each report line is prefixed with the wall-clock time."""
import sys
import time

import rclpy
from armdoc import doctor

_init = rclpy.init
rclpy.init = lambda **k: _init(args=["--ros-args", "-r", "/joint_states:=/joint_states_chaos"])


class Stamp:
    def write(self, s):
        for line in s.splitlines():
            sys.__stdout__.write(f"{time.time():.3f} {line}\n")
        sys.__stdout__.flush()

    def flush(self):
        pass


sys.stdout = Stamp()
doctor.main(sys.argv[1:])
