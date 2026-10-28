"""SI units; body x forward, y left, positive yaw counterclockwise."""
from dataclasses import dataclass
import math
import numpy as np


def solve_flat_polynomial(z0, zdot0, zf, zdotf, T):
    """Return coefficients in increasing power order, shape (4, 2)."""
    values = np.array([z0, zdot0, zf, zdotf], dtype=float)
    if values.shape != (4, 2) or not np.all(np.isfinite(values)):
        raise ValueError('Boundary values must be finite two-vectors')
    if not math.isfinite(T) or T <= 0:
        raise ValueError('T must be finite and positive')
    matrix = [[1, 0, 0, 0], [0, 1, 0, 0],
              [1, T, T*T, T**3], [0, 1, 2*T, 3*T*T]]
    return np.linalg.solve(matrix, values)


def evaluate(a, t):
    return (np.array([1, t, t*t, t**3]) @ a,
            np.array([0, 1, 2*t, 3*t*t]) @ a,
            np.array([0, 0, 2, 6*t]) @ a)


def flatness_inverse(velocity, acceleration):
    dx, dy = velocity
    ddx, ddy = acceleration
    norm2 = dx*dx + dy*dy
    if norm2 < 1e-12:
        return 0.0, 0.0  # Stationary endpoint: yaw is undefined; command stop.
    return math.sqrt(norm2), (dx*ddy-dy*ddx)/norm2


def horizon(distance):
    if not math.isfinite(distance) or distance <= 0.20:
        raise ValueError('Planning requires a target beyond the standoff')
    return max(4.0, 1.6*(distance-0.20)/0.16)


def wheel_rates(vx, wz):
    return np.array([vx+0.20*wz, vx-0.20*wz, vx-0.20*wz, vx+0.20*wz])/0.05


def motor_voltages(rates, previous, dt):
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('Derivative interval must be positive')
    return 0.38*np.asarray(rates)+0.06*(np.asarray(rates)-np.asarray(previous))/dt


@dataclass
class Observation:
    stamp: float
    distance: float
    bearing: float


class Supervisor:
    def __init__(self):
        self.state = 'SEARCHING'
        self.last_direction = 1.0
        self.start = None
        self.initial_distance = None
        self.coefficients = None
        self.T = None
        self.reason = ''

    def step(self, now, tag, tof=None, healthy=True):
        if self.state in ('ARRIVED', 'STOPPED_SHORT'):
            return 0.0, 0.0
        fresh = (tag is not None and 0 <= now-tag.stamp <= 0.45
                 and math.isfinite(tag.distance) and tag.distance > 0
                 and math.isfinite(tag.bearing))
        if (fresh and tag.distance <= 0.20) or (tof is not None and 0 < tof <= 0.20):
            self.state = 'ARRIVED'
            self.reason = 'camera' if fresh and tag.distance <= 0.20 else 'front_tof'
            return 0.0, 0.0
        if not healthy:
            self.state, self.start = 'SENSOR_HOLD', None
            return 0.0, 0.0
        if not fresh:
            self.state, self.start = 'SEARCHING', None
            return 0.0, 0.30*self.last_direction
        if abs(tag.bearing) > 1e-5:
            self.last_direction = math.copysign(1, tag.bearing)
        if abs(tag.bearing) > 0.18:
            self.state, self.start = 'ALIGNING', None
            return 0.0, float(np.clip(1.5*tag.bearing, -0.30, 0.30))
        if self.start is None:
            # Publish a stationary ALIGNING sample before starting the trajectory.
            self.start = now + 0.05
            self.initial_distance = tag.distance
            self.T = horizon(tag.distance)
            # The fixed initial speed and 4 s minimum can make a very short
            # cubic reverse. Do not turn its negative x derivative into forward
            # motion via the speed norm when reacquiring close to the target.
            if 6*(tag.distance-0.20)/self.T-2*0.06 < -1e-12:
                self.state, self.reason = 'STOPPED_SHORT', 'short_cubic_would_reverse'
                return 0.0, 0.0
            self.coefficients = solve_flat_polynomial(
                [0, 0], [0.06, 0], [tag.distance-0.20, 0], [0, 0], self.T)
            self.state = 'ALIGNING'
            return 0.0, 0.0
        self.state = 'CHASING'
        elapsed = max(0.0, now-self.start)
        if elapsed >= self.T:
            # No invented success: endpoint error requires a new, deliberate run.
            self.state, self.reason = 'STOPPED_SHORT', 'horizon_elapsed_without_standoff'
            return 0.0, 0.0
        pos, vel, acc = evaluate(self.coefficients, elapsed)
        v, w = flatness_inverse(vel, acc)
        # Small range feedback compensates tracking lag; full path remains cubic.
        error = (self.initial_distance-tag.distance)-pos[0]
        v = float(np.clip(v-0.5*error, 0, 0.16))
        # Slow smoothly near the guard; the guard itself always takes priority.
        v = min(v, math.sqrt(2*0.15*max(0, tag.distance-0.20)))
        w = float(np.clip(w+1.2*tag.bearing, -0.30, 0.30))
        return v, w
