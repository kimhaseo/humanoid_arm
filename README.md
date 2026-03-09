🦾 7-DOF Humanoid Arm — IK Control System
A Python-based control system for a 7-DOF robotic arm featuring real-time inverse kinematics, Cartesian interpolation, MeshCat 3D visualization, and CAN bus motor control via a GUI.

✨ Features

Real-time IK using Pinocchio (damped pseudo-inverse Jacobian)
Cartesian interpolation with LERP (position) + SLERP (orientation) across 500 waypoints
MeshCat 3D visualization synced to live motor state
Dark-themed tkinter GUI with end-effector position/orientation input, speed control, and scenario recording
CAN bus motor control over SLCAN serial adapter at 1 Mbps
Simulation mode — auto-falls back to DummyCanHandler when COM port is unavailable
Scenario playback — record multiple waypoints and execute them in sequence


📁 Project Structure
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

🔧 Hardware
ItemSpecDOF7Motor protocolCAN bus (SLCAN, 1 Mbps)Motor speed (fixed)1080 deg/sJoint range±180° (all joints)COM portCOM3 (configurable)

🗺️ Joint & Motor Mapping
Joint (IK)Motor NameCAN IDSignjoint_1left_joint10x141−1joint_2left_joint20x142+1joint_3left_joint30x143+1joint_4left_joint40x145+1joint_5left_joint50x144+1joint_6left_joint60x146+1joint_7left_joint70x147−1

⚠️ joint_4/joint_5의 CAN ID(0x145/0x144)는 물리 배선 기준으로 스왑되어 있습니다.


⚙️ IK Solver

Algorithm: Damped pseudo-inverse Jacobian (Levenberg-Marquardt)
Library: Pinocchio
End-effector frame: link_end
Interpolation: 500-step Cartesian LERP + SLERP from current pose to target
Tolerance: position < 1 mm, rotation < 0.57°

ParameterValueIK steps per waypoint20Random restarts3Interpolation steps500Damping factor1e-4Max joint step (dq)0.02 rad

🖥️ GUI Overview
Run with:
bashpython gui_controller.py
SectionDescription목표 위치End-effector X/Y/Z (meters)목표 자세Roll / Pitch / Yaw (degrees)이동 속도10–100% slider — adjusts dt per step (motor speed stays fixed at 1080 deg/s)IK 실행Compute IK and send motor commands시나리오Record current pose as waypoint → execute all in sequenceSIM / LIVE badgeShows whether real CAN or dummy mode is active
Speed Control
Motor hardware speed is always fixed at 1080 deg/s to preserve torque. Speed % only changes the inter-step delay (dt):
Speed %dt (s/step)Total move time (500 steps)10%0.025~12.5 s50%0.005~2.5 s100%0.0025~1.25 s

📦 Dependencies
bashpip install pinocchio meshcat numpy scipy python-can
PackagePurposepinocchioRobot kinematics / IK solvermeshcatBrowser-based 3D visualizationnumpyNumerical computationscipySLERP (Rotation interpolation)python-canCAN bus communicationtkinterGUI (Python standard library)

🚀 Quick Start
1. With real hardware (COM3 connected)
bashpython gui_controller.py
GUI title shows LIVE badge — motor commands are sent over CAN.
2. Simulation mode (no hardware)
COM port unavailable 시 자동으로 DummyCanHandler로 전환됩니다.
[CAN] COM3 연결 실패: ...
[CAN] → 더미 모드(시뮬레이션)로 전환합니다
GUI title shows SIM badge — all motor commands are printed to console only.
3. IK only (no GUI)
pythonfrom ik_solver_meshcat import IKSolver
import numpy as np

solver = IKSolver(meshcat=True)
target_pos = np.array([0.1, 0.2, -0.3])
target_rot = IKSolver.euler_to_rotation(0, 0, 0)

q_dict, pos_err, rot_err = solver.move_to(target_pos, target_rot, n_interp=500, dt=0.005)

🔌 CAN Bus Protocol
Motor commands use the Absolute Position Control command (0xA4):
Byte 0: 0xA4  (command)
Byte 1: 0x00  (null)
Byte 2: speed_low   (deg/s, little-endian)
Byte 3: speed_high
Byte 4: angle_low   (angle × 1000, little-endian 32-bit signed)
Byte 5: angle_mid1
Byte 6: angle_mid2
Byte 7: angle_high
