# rm_flatness_nav

ROS 2 package for autonomous AprilTag pursuit on a DJI RoboMaster EP using **differential flatness** trajectory generation, with real-time **motor armature voltage** telemetry.

Developed for **SCE 672: Advanced Robotics** (Term Project Assessment 2), King Fahd University of Petroleum & Minerals.

---

## Overview

The robot locates a user-selected AprilTag (T1, T2 or T3) lying flat on the floor 1.5–1.8 m ahead, aligns with it, cruises toward it along a smooth cubic trajectory, and brakes to a stop at a **20 cm standoff**. During the run, it logs the voltages of all four chassis motors every second to the terminal and to a CSV file.

### Mission behaviors

| State | Behavior |
|---|---|
| **SEARCHING** | Tag not seen for more than 0.45 s. The robot rotates in place (ω = ±0.30 rad/s) toward the side where the tag was last seen. |
| **ALIGNING** | Tag visible. The robot rotates in place (v = 0) until the bearing error is within ±10° (\|θ\| ≤ 0.18 rad). |
| **CHASING** | Tag centered. The robot follows the flatness-based cubic trajectory toward the tag. |
| **ARRIVED** | Active brake (v = 0, ω = 0) when the camera distance or front ToF reading is ≤ 0.20 m. |

The robotic arm is initialized to (X = 105 mm, Y = 30 mm). This tilts the gripper camera toward the floor without blocking the front ToF sensor.

---

## Method

### 1. Differential flatness inversion

The flat output is the planar position $`z(t) = [x(t),\ y(t)]^T`$. Linear and angular velocity are recovered algebraically:

```math
v(t) = \sqrt{\dot{x}(t)^2 + \dot{y}(t)^2}
```

```math
\omega(t) = \frac{\dot{x}(t)\,\ddot{y}(t) - \dot{y}(t)\,\ddot{x}(t)}{\dot{x}(t)^2 + \dot{y}(t)^2}
```

### 2. Cubic polynomial boundary-value solution

Each axis $`j \in \{x, y\}`$ is modeled as a cubic polynomial:

```math
z_j(t) = \alpha_0 + \alpha_1 t + \alpha_2 t^2 + \alpha_3 t^3
```

The coefficients are found with `numpy.linalg.solve` from:

```math
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
1 & T & T^2 & T^3 \\
0 & 1 & 2T & 3T^2
\end{bmatrix}
\begin{bmatrix}
\alpha_0 \\ \alpha_1 \\ \alpha_2 \\ \alpha_3
\end{bmatrix}
=
\begin{bmatrix}
z_j(0) \\ \dot{z}_j(0) \\ z_j(T) \\ \dot{z}_j(T)
\end{bmatrix}
```

Boundary conditions:

- **Start:** $`x(0) = 0`$, $`\dot{x}(0) = 0.06`$ m/s, $`y(0) = 0`$, $`\dot{y}(0) = 0`$
- **End:** $`x(T) = d_{tag} - 0.20`$ m, $`\dot{x}(T) = 0`$, $`y(T) = 0`$, $`\dot{y}(T) = 0`$

### 3. Time-horizon scaling

The horizon is chosen to keep cruising speed below $`v_{max} = 0.16`$ m/s:

```math
T = \max\left(4.0,\ \frac{1.6\,(d_0 - 0.20)}{v_{max}}\right)
```

### 4. Wheel kinematics

With wheel radius $`r = 0.05`$ m and chassis radius $`L_c = 0.20`$ m:

```math
\omega_1 = \omega_4 = \frac{v_x + L_c\,\omega_z}{r}
```

```math
\omega_2 = \omega_3 = \frac{v_x - L_c\,\omega_z}{r}
```

### 5. Motor armature voltage model

The model includes back-EMF ($`K_v = 0.38`$ V·s/rad) and inertial ($`K_a = 0.06`$ V·s²/rad) terms:

```math
V_i(t) = K_v\,\omega_i(t) + K_a\,\frac{\omega_i(t) - \omega_i(t - \Delta t)}{\Delta t}, \qquad i \in \{1, 2, 3, 4\}
```

---

## Package Structure

```
rm_flatness_nav/
├── rm_flatness_nav/
│   ├── driver_node.py        # RoboMaster connection, arm pose, camera & ToF
│   ├── planner_node.py       # FSM, flatness trajectory, voltage logging
│   └── ...
├── launch/
├── package.xml
├── setup.py
└── motor_voltage_log.csv     # Recorded telemetry (Target: Tag 3)
```
*(Update to match your actual files.)*

---

## Requirements

- ROS 2 (e.g. Humble)
- Python 3
- DJI RoboMaster Python SDK
- NumPy, OpenCV
- AprilTag detection library (e.g. `pupil-apriltags`)

## Build and Run

```bash
cd ~/ros2_ws/src
git clone <this-repo-url>
cd ~/ros2_ws
colcon build --packages-select rm_flatness_nav
source install/setup.bash

ros2 launch rm_flatness_nav <launch_file>.launch.py
```

## Output

`motor_voltage_log.csv` records the timestamp, the voltages V1–V4 and the FSM state once per second. After the robot halts at the 20 cm mark, all voltages read 0.000 V.

---

## Author

**Md Shakib Al Sifat**
Ph.D. in Aerospace Engineering, KFUPM

Course: SCE 672 – Advanced Robotics · Instructor: Prof. Aamer Iqbal Bhatti
