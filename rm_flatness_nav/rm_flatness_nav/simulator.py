"""Deterministic ideal kinematic plant. Output is explicitly SIMULATION."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Simulator(Node):
    def __init__(self):
        super().__init__('flatness_simulator')
        self.start = self.last = time.monotonic()
        self.x = self.y = self.yaw = 0.0
        self.command = dict(vx=0, wz=0, stamp=-1e9, state='WAITING')
        self.tag = self.create_publisher(String, '/rm/tag', 1)
        self.sensors = self.create_publisher(String, '/rm/sensors', 1)
        self.applied = self.create_publisher(String, '/rm/applied', 10)
        self.create_subscription(String, '/rm/command', self.receive, 1)
        self.create_timer(0.05, self.tick)

    def receive(self, msg):
        self.command = json.loads(msg.data)

    def tick(self):
        now = time.monotonic()
        dt, self.last = now-self.last, now
        c = self.command.copy()
        if now-c['stamp'] > 0.25:
            c.update(vx=0, wz=0, state='WATCHDOG')
        self.x += c['vx']*math.cos(self.yaw)*dt
        self.y += c['vx']*math.sin(self.yaw)*dt
        self.yaw += c['wz']*dt
        dx, dy = 1.65-self.x, -self.y
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)-self.yaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        if now-self.start > 2.0:  # Hidden initially to exercise search and alignment.
            self.tag.publish(String(data=json.dumps(dict(
                stamp=now, id=3, distance=distance, bearing=bearing))))
        self.sensors.publish(String(data=json.dumps(dict(stamp=now, healthy=True, enabled=True, tof_m=3.0))))
        c.update(stamp=now, source='SIMULATION', tag_distance_m=distance, tof_m=3.0,
                 x_m=self.x, y_m=self.y, yaw_rad=self.yaw,
                 vx_measured_mps=c['vx'], tag_bearing_rad=bearing)
        self.applied.publish(String(data=json.dumps(c)))


def main(args=None):
    rclpy.init(args=args)
    node = Simulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
