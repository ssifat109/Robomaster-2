import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .core import Observation, Supervisor, evaluate, flatness_inverse


class Planner(Node):
    def __init__(self):
        super().__init__('flatness_planner')
        self.control = Supervisor()
        self.tag = None
        self.sensors = None
        self.sensor_received = -1e9
        self.pub = self.create_publisher(String, '/rm/command', 1)
        self.create_subscription(String, '/rm/tag', self.on_tag, 1)
        self.create_subscription(String, '/rm/sensors', self.on_sensors, 1)
        self.create_timer(0.05, self.tick)

    def on_tag(self, msg):
        data = json.loads(msg.data)
        if data.get('id') == 3:
            self.tag = Observation(data['stamp'], data['distance'], data['bearing'])

    def on_sensors(self, msg):
        self.sensors = json.loads(msg.data)
        self.sensor_received = time.monotonic()

    def tick(self):
        now = time.monotonic()
        s = self.sensors or {}
        healthy = (now-self.sensor_received < 0.30 and s.get('healthy', False)
                   and s.get('enabled', False))
        tof = s.get('tof_m') if healthy else None
        v, w = self.control.step(now, self.tag, tof, healthy)
        reference = {}
        if self.control.coefficients is not None and self.control.start is not None:
            tau = min(self.control.T, max(0.0, now-self.control.start)) if self.control.start is not None else 0.0
            p, dp, ddp = evaluate(self.control.coefficients, tau)
            vr, wr = flatness_inverse(dp, ddp)
            reference = dict(plan_start=self.control.start, plan_time_s=tau,
                             horizon_s=self.control.T, initial_distance_m=self.control.initial_distance,
                             x_ref_m=float(p[0]), y_ref_m=float(p[1]),
                             v_ref_mps=vr, w_ref_radps=wr)
        self.pub.publish(String(data=json.dumps(dict(
            stamp=now, vx=v, wz=w, state=self.control.state, reason=self.control.reason,
            reference=reference))))


def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
