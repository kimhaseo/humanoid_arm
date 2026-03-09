# 🦾 7-DOF Humanoid Arm — IK Control System

A Python-based control system for a 7-DOF robotic arm featuring real-time inverse kinematics, Cartesian interpolation, MeshCat 3D visualization, and CAN bus motor control via a GUI.

---

## ✨ Features

- **Real-time IK** using Pinocchio (damped pseudo-inverse Jacobian)
- **Cartesian interpolation** with LERP (position) + SLERP (orientation) across 500 waypoints
- **MeshCat 3D visualization** synced to live motor state
- **Dark-themed tkinter GUI** with end-effector position/orientation input, speed control, and scenario recording
- **CAN bus motor control** over SLCAN serial adapter at 1 Mbps
- **Simulation mode** — auto-falls back to `DummyCanHandler` when COM port is unavailable
- **Scenario playback** — record multiple waypoints and execute them in sequence

---

## 📁 Project Structure

```
humanoid_arm/
├── gui_controller.py        # Main GUI (tkinter dark theme, IK trigger, scenario)
├── ik_solver_meshcat.py     # IK solver + MeshCat visualizer (Pinocchio)
├── motor_controller.py      # CAN bus motor command handler
├── can_handler.py           # CAN interface + DummyCanHandler fallback
├── config/
│   └── motor_cmd.py         # AngleCommand / PidCommand / AeccelCommand dataclasses
└── 7dof_arm_urdf/
    ├── 7dof_arm_urdf.urdf   # Robot URDF model
    └── meshes/              # STL mesh files for visualization
```

---

## 🔧 Hardware

| Item | Spec |
|------|------|
| DOF | 7 |
| Motor protocol | CAN bus (SLCAN, 1 Mbps) |
| Motor speed (fixed) | **1080 deg/s** |
| Joint range | ±180° (all joints) |
| COM port | COM3 (configurable) |

---

## 🗺️ Joint & Motor Mapping

| Joint (IK) | Motor Name | CAN ID | Sign |
|---|---|---|---|
| joint_1 | left_joint1 | `0x141` | −1 |
| joint_2 | left_joint2 | `0x142` | +1 |
| joint_3 | left_joint3 | `0x143` | +1 |
| joint_4 | left_joint4 | `0x145` | +1 |
| joint_5 | left_joint5 | `0x144` | +1 |
| joint_6 | left_joint6 | `0x146` | +1 |
| joint_7 | left_joint7 | `0x147` | −1 |

> ⚠️ joint_4/joint_5의 CAN ID(`0x145`/`0x144`)는 물리 배선 기준으로 스왑되어 있습니다.

---

## ⚙️ IK Solver

- **Algorithm**: Damped pseudo-inverse Jacobian (Levenberg-Marquardt)
- **Library**: [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
- **End-effector frame**: `link_end`
- **Interpolation**: 500-step Cartesian LERP + SLERP from current pose to target
- **Tolerance**: position < 1 mm, rotation < 0.57°

| Parameter | Value |
|---|---|
| IK steps per waypoint | 20 |
| Random restarts | 3 |
| Interpolation steps | 500 |
| Damping factor | 1e-4 |
| Max joint step (dq) | 0.02 rad |

---

## 🖥️ GUI Overview

Run with:

```bash
python gui_controller.py
```

| Section | Description |
|---|---|
| **목표 위치** | End-effector X/Y/Z (meters) |
| **목표 자세** | Roll / Pitch / Yaw (degrees) |
| **이동 속도** | 10–100% slider — adjusts `dt` per step (motor speed stays fixed at 1080 deg/s) |
| **IK 실행** | Compute IK and send motor commands |
| **시나리오** | Record current pose as waypoint → execute all in sequence |
| **SIM / LIVE badge** | Shows whether real CAN or dummy mode is active |

### Speed Control

Motor hardware speed is **always fixed at 1080 deg/s** to preserve torque. Speed % only changes the inter-step delay (`dt`):

| Speed % | dt (s/step) | Total move time (500 steps) |
|---|---|---|
| 10% | 0.025 | ~12.5 s |
| 50% | 0.005 | ~2.5 s |
| 100% | 0.0025 | ~1.25 s |

---

## 📦 Dependencies

```bash
pip install pinocchio meshcat numpy scipy python-can
```

| Package | Purpose |
|---|---|
| `pinocchio` | Robot kinematics / IK solver |
| `meshcat` | Browser-based 3D visualization |
| `numpy` | Numerical computation |
| `scipy` | SLERP (Rotation interpolation) |
| `python-can` | CAN bus communication |
| `tkinter` | GUI (Python standard library) |

---

## 🚀 Quick Start

### 1. With real hardware (COM3 connected)
```bash
python gui_controller.py
```
GUI title shows **`LIVE`** badge — motor commands are sent over CAN.

### 2. Simulation mode (no hardware)
COM port unavailable 시 자동으로 `DummyCanHandler`로 전환됩니다.
```
[CAN] COM3 연결 실패: ...
[CAN] → 더미 모드(시뮬레이션)로 전환합니다
```
GUI title shows **`SIM`** badge — all motor commands are printed to console only.

### 3. IK only (no GUI)
```python
from ik_solver_meshcat import IKSolver
import numpy as np

solver = IKSolver(meshcat=True)
target_pos = np.array([0.1, 0.2, -0.3])
target_rot = IKSolver.euler_to_rotation(0, 0, 0)

q_dict, pos_err, rot_err = solver.move_to(target_pos, target_rot, n_interp=500, dt=0.005)
```

---

## 🔌 CAN Bus Protocol

Motor commands use the **Absolute Position Control** command (`0xA4`):

```
Byte 0: 0xA4  (command)
Byte 1: 0x00  (null)
Byte 2: speed_low   (deg/s, little-endian)
Byte 3: speed_high
Byte 4: angle_low   (angle × 1000, little-endian 32-bit signed)
Byte 5: angle_mid1
Byte 6: angle_mid2
Byte 7: angle_high
```

---

## 📝 License

MIT
