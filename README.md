# Bicopter Control Simulation (Simulink)

This repository contains a dynamic simulation and control implementation
for a tilt-rotor bicopter UAV developed in MATLAB/Simulink.

------------------------------------------------------------------------

## Overview

The model implements:

-   6DOF rigid body dynamics (Aerospace Blockset)
-   Quaternion-based orientation representation
-   Cascaded control structure:
    -   Attitude controller (outer loop)
    -   Angular rate controller (inner loop)
-   Nonlinear control allocation (actuator mapping)
-   Differential thrust and tilt-based control

------------------------------------------------------------------------

## Architecture

High-level structure:

    Desired attitude (phi, theta, psi)
            ↓
    Attitude Controller
            ↓ (desired rates: p_cmd, q_cmd, r_cmd)
    Rate PID Controller
            ↓ (desired moments: Mx, My, Mz)
    Control Allocation
            ↓ (u1, u2, alpha1, alpha2)
    Airframe Dynamics (6DOF)

------------------------------------------------------------------------

## Dynamics Model

-   Implemented using Aerospace Blockset 6DOF (Quaternion) block
-   Body-fixed coordinate frame
-   Gravity applied explicitly
-   Reaction torque model for yaw dynamics
-   Thrust vector rotates in the x--z plane via tilt servos

Parameters:

-   Arm length: 0.5 m
-   Mass: 0.25 kg
-   Max thrust per motor: 350 g (\~3.43 N)

------------------------------------------------------------------------

## Control Design

### Attitude Controller (Outer Loop)

Inputs:

-   Desired Euler angles (roll, pitch, yaw)
-   Current attitude from state estimation

Outputs:

-   Desired body rates (p_cmd, q_cmd, r_cmd)

Implementation:

-   Proportional control on angle error
-   Yaw error wrapped to \[-π, π\]

------------------------------------------------------------------------

### Rate Controller (Inner Loop)

Inputs:

-   Desired body rates
-   Measured body rates from 6DOF output

Outputs:

-   Desired moments (Mx, My, Mz)

Implementation:

-   PID control per axis
-   Anti-windup enabled
-   Saturation applied to output moments

------------------------------------------------------------------------

### Control Allocation

Maps desired wrench into actuator commands.

Inputs:

-   Desired vertical force Fz
-   Desired roll moment Mx
-   Desired yaw moment Mz

Outputs:

-   Motor commands (u1, u2)
-   Tilt angles (alpha1, alpha2)

Key ideas:

-   Roll moment generated via differential thrust
-   Yaw moment generated via differential tilt
-   Common tilt used for force vector direction

Allocator uses geometric relationships rather than fixed mixer matrices
due to nonlinear thrust vectoring.

------------------------------------------------------------------------

## Solver Configuration

-   Fixed-step solver
-   Discrete controller implementation
-   Control loop sample rate typical of UAV flight controllers

------------------------------------------------------------------------

## How to Run

1.  Open MATLAB.
2.  Open the Simulink model:

    BiCopterModel.slx

3.  Run simulation.

Recommended starting conditions:

-   Hover configuration
-   Zero attitude commands

------------------------------------------------------------------------

## Development Notes

This project explores differences between quadcopter and bicopter
control:

-   Control allocation is nonlinear.
-   Control authority varies with tilt angle.
-   Actuator bandwidth asymmetry (motors vs tilt servos) influences
    tuning.

------------------------------------------------------------------------


## License

MIT License
