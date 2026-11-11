"""ROS 2 side: relay joint states through the active faults, and drive controller_manager faults.

Safety: a controller armchaos deactivated is always put back. On Ctrl-C, SIGTERM, an error, or the
--max-duration guard, run() reactivates it (blocking, with retries) before it returns, and reports any
controller it could not restore."""
import json
import signal
import time
from dataclasses import dataclass, field

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from armchaos.faults import Sample
from armchaos.scenario import Engine


@dataclass
class RunResult:
    stopped_by: str = None                       # None (finished), SIGINT, SIGTERM or max-duration
    unrestored: list = field(default_factory=list)
    windows: dict = field(default_factory=dict)  # id(step) -> [wall start, wall stop]


class Relay(Node):
    def __init__(self, steps, source, out, cm, events_path=None, lead=0.0, max_duration=600.0):
        super().__init__("armchaos")
        self.engine, self.cm = Engine(steps), cm.rstrip("/")
        self.lead, self.max_duration = lead, max_duration
        self.pub = self.create_publisher(JointState, out, qos_profile_sensor_data)
        self.create_subscription(JointState, source, self.on_js, qos_profile_sensor_data)
        self.sw = self.create_client(SwitchController, f"{self.cm}/switch_controller")
        self.t0 = time.monotonic()
        self.events = open(events_path, "w") if events_path else None
        self.done = False
        self.expired = False
        self.deactivated = set()                 # controllers armchaos switched off and has not yet switched back on
        self.windows = {}
        self.create_timer(0.01, self.tick)

    def now(self):
        """Scenario time: negative during the healthy lead-in, 0 when the first fault may start."""
        return time.monotonic() - self.t0 - self.lead

    def log(self, kind, step):
        line = f"t={self.now():5.1f}s {kind:5} {step.label()}"
        print(line, flush=True)
        if self.events:
            self.events.write(json.dumps({"t": self.now(), "wall": time.time(), "event": kind.strip(),
                                          "fault": step.fault.name, "label": step.label()}) + "\n")
            self.events.flush()

    def mark(self, step, started):
        w = self.windows.setdefault(id(step), [time.time(), None])
        if not started:
            w[1] = time.time()

    def close_windows(self):
        for st in self.engine.steps:
            if st.active:
                self.mark(st, False)

    # controller faults -------------------------------------------------------------------------
    def _call(self, controller, activate):
        req = SwitchController.Request()
        (req.activate_controllers if activate else req.deactivate_controllers).append(controller)
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        req.timeout = Duration(sec=5)
        if not self.sw.service_is_ready():
            self.get_logger().error(f"{self.cm}/switch_controller is not available")
            return None
        return self.sw.call_async(req)

    def switch(self, controller, activate):
        if not activate:
            self.deactivated.add(controller)     # remembered from the request, so a lost reply still gets restored
        fut = self._call(controller, activate)
        if fut is not None:
            fut.add_done_callback(lambda f, c=controller, a=activate: self._switched(c, a, f))

    def _switched(self, controller, activate, fut):
        try:
            ok = bool(fut.result().ok)
        except Exception:
            ok = False
        if activate and ok:
            self.deactivated.discard(controller)
        elif not ok:
            self.get_logger().error(f"switching {controller} {'on' if activate else 'off'} was refused")

    def restore_all(self, timeout=8.0, tries=3):
        """Blocking: reactivate every controller still deactivated. Returns the ones that could not be restored."""
        failed = []
        for c in sorted(self.deactivated):
            ok = False
            for _ in range(tries):
                if not self.sw.wait_for_service(timeout_sec=timeout / tries):
                    continue
                fut = self._call(c, True)
                if fut is None:
                    continue
                rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
                try:
                    ok = fut.done() and bool(fut.result().ok)
                except Exception:
                    ok = False
                if ok:
                    break
            if ok:
                self.deactivated.discard(c)
                print(f"restored controller {c}", flush=True)
            else:
                failed.append(c)
        return failed

    # stream faults -----------------------------------------------------------------------------
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
        if t + self.lead > self.max_duration:      # guard: nothing may run longer than --max-duration
            self.expired = self.done = True
            return
        started, stopped = self.engine.update(t)
        for st in started:
            self.log("START", st)
            self.mark(st, True)
            if st.fault.controller:
                self.switch(st.fault.p["controller"], False)
        for st in stopped:
            self.log("STOP", st)
            self.mark(st, False)
            if st.fault.controller:
                self.switch(st.fault.p["controller"], True)
        self.emit(self.engine.tick(t))
        if t > self.engine.end() + 0.5:
            self.done = True


def run(steps, source, out, cm, events_path=None, linger=6.0, lead=0.0, max_duration=600.0,
        restore_on_exit=True, watcher=None):
    """Run the scenario. Returns a RunResult. Ctrl-C and SIGTERM stop it cleanly."""
    rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions.NO)   # we handle SIGINT ourselves
    got = {"sig": None}

    def on_signal(signum, frame):
        if got["sig"] is None:
            got["sig"] = signal.Signals(signum).name
    old = {s: signal.signal(s, on_signal) for s in (signal.SIGINT, signal.SIGTERM)}
    node = Relay(steps, source, out, cm, events_path, lead, max_duration)
    result = RunResult()
    try:
        if watcher:
            watcher.start()
        while rclpy.ok() and not node.done and not got["sig"]:
            rclpy.spin_once(node, timeout_sec=0.05)
        end = time.monotonic() + linger          # keep relaying cleanly so observers see the recovery
        while rclpy.ok() and not got["sig"] and not node.expired and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.close_windows()
        result.stopped_by = got["sig"] or ("max-duration" if node.expired else None)
        if result.stopped_by:
            print(f"stopping ({result.stopped_by})", flush=True)
        if watcher:
            watcher.stop()
        if node.deactivated and (restore_on_exit or result.stopped_by is None):
            print("restoring controllers, please wait", flush=True)
            result.unrestored = node.restore_all()
        elif node.deactivated:
            print(f"left deactivated (--no-restore-on-exit): {', '.join(sorted(node.deactivated))}", flush=True)
        if result.unrestored:
            print(f"COULD NOT RESTORE: {', '.join(result.unrestored)} (switch them on by hand)", flush=True)
        result.windows = node.windows
        for s, h in old.items():
            signal.signal(s, h)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result
