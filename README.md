# 4-DOF 3D Robotic Arm Simulator

## Overview
This project uses forward kinematics for a 4-DOF robotic arm using Denavit-Hartenberg (DH) parameters. The arm can be visualised in 3D using Matplotlib.

## How it works

## Denavit-Hartenberg (DH) Parameters - My understanding

## What are DH parameters
DH parameters are four numbers that describe the relationship between teo adjacent joints in a robotic arm. They are:
- Theta: The joint angle. This is the variable we can so we can move the arm
- d: The link offset. This is the distance between the joint axis between links
- a: The link length. The distance between joint axes
- Alpha: The link twist. The rotation between joint axes

 ## How I used them
 I built a 4-DOF planar arm and extended it to 3D by adding alpha twists. I used the DH parameters to build transformation matrices for each joint, the chained them together using matrix multiplication to get the end-effector position.

 ## Why this matters
 Understanding DH parameters is the foundation of robotic kinematics. Without this, you cannot find and calculate where the robot's hand is in space.

 ## My code
I implemented 'dh_transform()' and 'forward_kinematics()' to calculate and find out the end-effector position. I verified the code against my own hand calculations for 2D,3D and 4D arms.
