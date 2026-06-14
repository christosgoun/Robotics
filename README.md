# UR10 Door Opening Simulation

## Overview
This repository contains a MATLAB-based simulation of a Universal Robots UR10 manipulator performing a dynamic door-opening task. The project is divided into two main phases: environmental trajectory generation and manipulator inverse kinematics simulation. 

The system models the physical constraints of a hinged door and calculates the necessary joint angles and velocities for the UR10 to track the door handle seamlessly.

## Repository Structure
*   **`PartA.m`**: Generates the kinematic trajectory of the door and handle over a 5-second period, broken down into knob-twist and door-swing phases. Exports the spatial transformations[cite: 7].
*   **`PartB.m`**: The core simulation script. It loads the pre-computed trajectories and solves the inverse kinematics for the UR10 to track the handle frame `H` in real-time[cite: 2].
*   **`ur10robot.m`**: Defines the UR10 robotic arm model using Denavit-Hartenberg (DH) parameters, including center of mass and link mass data[cite: 4].
*   **`partA_H_trajectory.mat`**: Exported MAT-file containing the 4x4 homogenous transformation matrices of the handle frame over time[cite: 1, 7].
*   **`partA_D_trajectory.mat`**: Exported MAT-file containing the 4x4 homogenous transformation matrices of the door frame over time[cite: 1, 7].
*   **`debug.m`**: Auxiliary script for troubleshooting workspace and matrix dimension errors.
*   **`.gitignore`**: Ignores standard MATLAB temporary files (`*.asv`, `*.slxc`, etc.) and system clutter.

## Prerequisites
To run this simulation, you need:
1.  **MATLAB** (R2021a or newer recommended).
2.  **Robotics Toolbox for MATLAB** (by Peter Corke). The simulation heavily relies on `SerialLink`, `fkine`, `jacobe`, and spatial math functions like `transl` and `trotx`[cite: 2, 4, 7].

## Usage
1.  **Generate Trajectories:** Run `PartA.m` first. This will calculate the spatial paths and save `partA_H_trajectory.mat` and `partA_D_trajectory.mat` to your current directory[cite: 7].
2.  **Run Simulation:** Execute `PartB.m`. The script will load the MAT-files, calculate the required joint variables via the Jacobian pseudo-inverse, and display a 3D animation of the UR10 opening the door, alongside joint position/velocity plots[cite: 2].

## System Kinematics
*   **Door & Handle**: The door is modeled with a width of 1.0m and height of 2.0m[cite: 7]. The handle trajectory includes a -45° twist (first 2 seconds) and a -30° door swing (subsequent 3 seconds)[cite: 7].
*   **Manipulator**: The end-effector target frame `{e}` is defined with a constant static offset from the handle frame `{H}` via a predefined hand-eye transformation matrix[cite: 2].

## ⚠️ Known Limitations & Technical Risks
*   **Singularity Handling**: The inverse kinematics solution relies on the unweighted Moore-Penrose pseudo-inverse of the geometric Jacobian (`pinv(J)`)[cite: 2]. **Risk**: If the prescribed trajectory forces the arm near a singular configuration, joint velocities will spike unacceptably, causing computational instability and theoretical mechanical failure. 
*   **Dynamic Modeling**: This simulation is strictly kinematic. Inertial forces, friction, and torque limits of the UR10 motors are not evaluated.
*   **Collision Detection**: There is no active self-collision or environmental collision avoidance implemented. The success of the simulation relies entirely on the pre-planned initial conditions.
