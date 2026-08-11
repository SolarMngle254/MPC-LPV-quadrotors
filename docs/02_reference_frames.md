# 02. Reference Frame Definitions

## 2.1 Overview

The quadrotor model uses two primary coordinate frames:

- Earth-fixed Inertial Frame
- Body-fixed Frame

The inertial frame is used to describe global position and
translational motion, while the body-fixed frame is used for
attitude dynamics and actuator-related quantities.

![Drone Orientation and Reference Frames](../pics/reference_frames.png)

*Figure: Drone Orientation and Reference Frames.*


## 2.2 Earth-fixed Inertial Frame

The Earth-fixed inertial frame is defined as

\[
\mathcal{F}_E = \{O_E, X_E, Y_E, Z_E\}.
\]

The frame is fixed to the ground and follows the right-hand rule.
The East-North-Up (ENU) convention is adopted, with \(Z_E\)
representing the upward direction.

The quadrotor position is expressed in this frame as

\[
\mathbf{p}
=
[x,\ y,\ z]^T.
\]

The inertial frame is the working frame for the position controller
and global trajectory tracking.


## 2.3 Body-fixed Frame

The body-fixed frame is attached to the quadrotor, with its origin
located at the center of mass.

The body axes are defined as:

- \(X_B\): forward direction
- \(Y_B\): right direction
- \(Z_B\): body vertical axis

The body-fixed frame is used to describe thrust, moments, and
angular velocity:

\[
\boldsymbol{\omega}
=
[p,\ q,\ r]^T.
\]

It is also the working frame for the attitude controller.


## 2.4 Coordinate Transformation

Quantities such as velocity, force, and acceleration can be
transformed between the body-fixed and inertial frames using
coordinate transformation matrices.

The rotation matrix

\[
R(\eta)
\]

maps vectors from the body-fixed frame to the inertial frame.

The angular velocity transformation matrix

\[
T(\eta)
\]

relates body angular velocity to the time derivatives of the
Euler angles.

The vehicle attitude is described by the Euler angles:

\[
\phi,\quad \theta,\quad \psi
\]

corresponding to roll, pitch, and yaw.

A standard \(Z-Y-X\) yaw-pitch-roll rotation sequence is adopted.

![Euler Angle Formation](../pic/euler_angle_formation.png)

*Figure: Euler Angle Formation via Successive Rotations.*


## 2.5 Role in the Dynamic Model

The two reference frames provide the basis for separating the
quadrotor dynamics into:

- Translational dynamics in the inertial frame
- Rotational dynamics in the body-fixed frame

This separation is used in the subsequent development of the
quadrotor dynamic model and the hierarchical control architecture.
