"""ROS 2 side: relay joint states through the active faults, and drive controller_manager faults."""
import json
import time

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from armchaos.faults import Sample
from armchaos.scenario import Engine


class Relay(Node):
    def __init__(self, steps, source, out, cm, events_path=None):
        super().__init__("armchaos")
        self.engine, self.cm = Engine(steps), cm.rstrip("/")
        self.pub = self.create_publisher(JointState, out, qos_profile_sensor_data)
        self.create_subscription(JointState, source, self.on_js, qos_profile_sensor_data)
        self.sw = self.create_client(SwitchController, f"{self.cm}/switch_controller")
        self.t0 = time.monotonic()
        self.events = open(events_path, "w") if events_path else None
        self.done = False
        self.create_timer(0.01, self.tick)

    def now(self):
        return time.monotonic() - self.t0

    def log(self, kind, step):
        line = f"t={self.now():5.1f}s {kind:5} {step.label()}"
        print(line, flush=True)
        if self.events:
            self.events.write(json.dumps({"t": self.now(), "wall": time.time(), "event": kind.strip(),
                                          "fault": step.fault.name, "label": step.label()}) + "\n")
            self.events.flush()

    def switch(self, controller, activate):
        req = SwitchController.Request()
        (req.activate_controllers if activate else req.deactivate_controllers).append(controller)
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        req.timeout = Duration(sec=5)
        if not self.sw.service_is_ready():
            self.get_logger().error(f"{self.cm}/switch_controller is not available")
            return
        self.sw.call_async(req)

    def emit(self, samples):
        for s in samples:
            m = JointState(name=list(s.names), position=list(s.pos), velocity=list(s.vel), effort=list(s.eff))
            m.header.stamp = s.stamp or self.get_clock().now().to_msg()
            self.pub.publish(m)

    def on_js(self, m):
        s = Sample(list(m.name), list(m.position), list(m.velocity), list(m.effort), m.header.stamp)
        self.emit(self.engine.process(self.now(), s))

    def tick(self):
        t = self.now()
        started, stopped = self.engine.update(t)
        for st in started:
            self.log("START", st)
            if st.fault.controller:
                self.switch(st.fault.p["controller"], False)
        for st in stopped:
            self.log("STOP", st)
            if st.fault.controller:
                self.switch(st.fault.p["controller"], True)
        self.emit(self.engine.tick(t))
        if t > self.engine.end() + 0.5:
            self.done = True


def run(steps, source, out, cm, events_path=None, linger=6.0):
    rclpy.init()
    node = Relay(steps, source, out, cm, events_path)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
        end = time.monotonic() + linger          # keep relaying cleanly so observers see the recovery
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        for st in node.engine.steps:             # restore anything still broken
            if st.active and st.fault.controller:
                node.switch(st.fault.p["controller"], True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
