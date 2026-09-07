# 4-DOF Robotic Arm Simulator

## Overview

This project is a **4-DOF (Degree of Freedom) robotic arm simulator** built from first principles using Python. It demonstrates forward kinematics, inverse kinematics, physics simulation, and PID control in a 3D environment.

The arm is modelled using **Denavit-Hartenberg (DH) parameters** and visualised in 3D using matplotlib. The Physics simulation is handled by **PyBullet**, and the arm can track target trajectories using **PID control**.

This project was built as part of my portfolio for UCL's MSc in Medical Robotics and AI.


## Video of 4 DOF Arm:
[![Watch the Robotic Arm Demo](https://img.youtube.com/vi/VVV-HzRpLww/0.jpg)](https://www.youtube.com/watch?v=VVV-HzRpLww)

---

## Features

| Feature | Description |
|---------|-------------|
| **Forward Kinematics (FK)** | Calculates the end-effector position from joint angles using DH parameters |
| **Inverse Kinematics (IK)** | Calculates joint angles to reach a target position (analytical + iterative) |
| **PyBullet Physics** | Simulates gravity, mass, inertia, and collisions |
| **PID Control** | Tracks target trajectories with smooth motion |
| **3D Visualisation** | Interactive 3D plot showing the arm in motion |
| **Verification** | Hand-calculated FK verified against code output |

---

## Tech Stack

| Tool | Purpose |
|------|---------|
| Python 3.x | Core language |
| NumPy | Matrix operations, DH transformations |
| Matplotlib | 3D visualisation |
| PyBullet | Physics simulation |
| Git | Version control |

---

## How It Works

### 1. Denavit-Hartenberg (DH) Parameters

The arm is described using DH parameters. Each joint is defined by four parameters:

| Parameter | Description |
|-----------|-------------|
| `θ (theta)` | Joint angle (rotation around Z-axis) |
| `d` | Link offset (translation along Z-axis) |
| `a` | Link length (translation along X-axis) |
| `α (alpha)` | Link twist (rotation around X-axis) |

These parameters are used to build 4x4 homogeneous transformation matrices for each joint.

### 2. Forward Kinematics

The transformation matrices for all joints are multiplied together to get the end-effector position.
T_total = T_1 * T_2 * T_3 * T_4


### 3. Inverse Kinematics

Given a target (x, y, z) position, the IK solver:
- Uses an **analytical method** for the 2-joint case
- Uses **Cyclic Coordinate Descent (CCD)** for the 4-joint case

### 4. Physics Simulation

PyBullet handles:
- Gravity
- Link mass and inertia
- Collision detection
- Realistic joint dynamics

### 5. PID Control

A PID controller is used to track target trajectories smoothly.

---

## Project Structure
robot-arm-simulator/

├── README.md # This file

├── requirements.txt # Dependencies

├── fk_arm.py # Forward kinematics (DH + FK)

├── ik_arm.py # Inverse kinematics (2-joint analytical)

├── ik_arm_3dof.py # Inverse kinematics (3-DOF iterative)

├── ik_bullet.py # IK applied in PyBullet simulation

├── load_arm.py # Loads URDF and sets up PyBullet

├── planar_4dof.urdf # URDF file for the 4-DOF arm

├── pid.py # PID controller


---

## How to Run

### 1. Clone the Repository

```bash
git clone https://github.com/mogozv/robot-arm-simulator.git
cd robot-arm-simulator
2. Install Dependencies
bash
pip install -r requirements.txt
3. Run Forward Kinematics
bash
python fk_arm.py
4. Run Inverse Kinematics
bash
python ik_arm.py
5. Run PyBullet Simulation
bash
python load_arm.py
Results
Forward Kinematics – Verification
Configuration	Hand-Calculated	Code Output	Match
(0°, 0°, 0°, 0°)	(2.8, 0.0, 0.0)	(2.8, 0.0, 0.0)	✅
(30°, 45°, 15°, -30°)	(1.27, 2.23, 0.0)	(1.27, 2.23, 0.0)	✅

What I Learned:
DH parameters are the foundation of robot kinematics

Matrix multiplication chains transformations together

Inverse kinematics is harder than forward kinematics

PyBullet makes physics simulation accessible

PID control requires tuning – too much gain causes oscillation

Future Work:
Sim-to-Real Integration: Extend the simulation to control a physical robotic arm, validating the kinematics and control algorithms on hardware and enabling real-time feedback

Obstacle avoidance using RRT or A*

Path planning for complex trajectories

6-DOF extension for more realistic applications

ROS integration for real robot control

Author
Mohammed Godir – Computer Science student at the University of Westminster


LinkedIn:

(https://www.linkedin.com/in/mohammed-godir/)



Copyright (c) 2026 Mohammed Godir

Permission is hereby granted, free of charge, to any person obtaining a copy


