"""One-second model estimates plus high-rate evidence, never measured voltages."""
import csv
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .core import wheel_rates, motor_voltages


class VoltageLogger(Node):
    def __init__(self):
        super().__init__('motor_voltage_logger')
        self.declare_parameter('output_dir', 'data/run')
        self.folder = Path(self.get_parameter('output_dir').value)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.fields = ['t_s', 'V1_V', 'V2_V', 'V3_V', 'V4_V', 'state',
                       'vx_mps', 'wz_radps', 'tag_distance_m', 'tof_m',
                       'source', 'voltage_basis', 'dt_s', 'command_acknowledged',
                       'plan_start', 'plan_time_s', 'horizon_s', 'initial_distance_m',
                       'x_ref_m', 'y_ref_m', 'v_ref_mps', 'w_ref_radps',
                       'range_progress_m', 'range_tracking_error_m', 'vx_measured_mps',
                       'velocity_tracking_error_mps', 'tag_bearing_rad']
        self.files = [self.folder.joinpath(name).open('x', newline='') for name in
                      ('motor_voltage_log.csv', 'execution.csv')]
        self.writers = [csv.DictWriter(f, fieldnames=self.fields) for f in self.files]
        for writer in self.writers:
            writer.writeheader()
        self.events = (self.folder/'raw_telemetry.jsonl').open('x')
        self.start = self.last = None
        self.previous = np.zeros(4)
        self.next_second = 0
        self.create_subscription(String, '/rm/applied', self.receive, 10)

    def receive(self, msg):
        data = json.loads(msg.data)
        now = data['stamp']
        self.events.write(json.dumps(data)+'\n')
        self.events.flush()
        rates = wheel_rates(data['vx'], data['wz'])
        if self.start is None:
            self.start = now
            volts, dt = np.full(4, np.nan), 0.0
            # Only a known stationary first sample can establish zero derivative.
            if np.all(rates == 0):
                volts[:] = 0
        else:
            dt = now-self.last
            if dt <= 0:
                return
            volts = motor_voltages(rates, self.previous, dt)
        self.previous, self.last = rates, now
        t = now-self.start
        row = dict(t_s=f'{t:.6f}', state=data['state'], vx_mps=data['vx'],
                   wz_radps=data['wz'], tag_distance_m=data.get('tag_distance_m', ''),
                   tof_m=data.get('tof_m', ''), source=data['source'],
                   voltage_basis='command_kinematics_model', dt_s=f'{dt:.6f}')
        row.update({f'V{i+1}_V': f'{v:.6f}' for i, v in enumerate(volts)})
        ref = data.get('reference', {})
        row.update({key: value for key, value in ref.items() if key in self.fields})
        row['command_acknowledged'] = data.get('command_acknowledged', '')
        row['tag_bearing_rad'] = data.get('tag_bearing_rad', '')
        measured_v = data.get('vx_measured_mps')
        row['vx_measured_mps'] = measured_v if measured_v is not None else ''
        if measured_v is not None and 'v_ref_mps' in ref:
            row['velocity_tracking_error_mps'] = ref['v_ref_mps']-measured_v
        if data.get('tag_distance_m') is not None and 'initial_distance_m' in ref:
            progress = ref['initial_distance_m']-data['tag_distance_m']
            row['range_progress_m'] = progress
            row['range_tracking_error_m'] = ref['x_ref_m']-progress
        self.writers[1].writerow(row)
        if t >= self.next_second:
            self.writers[0].writerow(row)
            self.next_second = int(t)+1
            self.get_logger().info(f'{t:6.2f}s | '+ ' '.join(f'{v:8.3f}' for v in volts)
                                   + f' V | {data["state"]} | {data["source"]}')
        for f in self.files:
            f.flush()

    def destroy_node(self):
        for f in self.files:
            f.close()
        self.events.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoltageLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
