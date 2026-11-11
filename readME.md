1. The Real 6-DOF Drone Dynamics ($\dot{x} = f(x, u)$)

A 6-DOF drone state vector $\mathbf{x} \in \mathbb{R}^{13}$ consists of:

$$
\mathbf{x} = \begin{bmatrix} \mathbf{p} \\ \mathbf{v} \\ \mathbf{q} \\ \mathbf{\omega} \end{bmatrix}^T = \begin{bmatrix} \text{position}(3) \\ \text{velocity}(3) \\ \text{quaternion}(4) \\ \text{body angular vel}(3) \end{bmatrix}^T
$$

And control inputs $\mathbf{u} = \begin{bmatrix} T & \mathbf{\tau} \end{bmatrix}^T = \begin{bmatrix} \text{total thrust} & \text{body torques}(3) \end{bmatrix}^T$.

Using Euler's equations of motion for a rigid body with inertia tensor $\mathbf{J}$:

$$
\begin{aligned}
\dot{\mathbf{p}} &= \mathbf{v} \\
\dot{\mathbf{v}} &= \mathbf{g} + \frac{1}{m} \mathbf{R}(\mathbf{q}) \begin{bmatrix} 0 \\ 0 \\ T \end{bmatrix} - \mathbf{K}_d \mathbf{v} \quad (\text{Translational Drag}) \\
\dot{\mathbf{q}} &= \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix} 0 \\ \mathbf{\omega} \end{bmatrix} \\
\dot{\mathbf{\omega}} &= \mathbf{J}^{-1} \left( \mathbf{\tau} - \mathbf{\omega} \times (\mathbf{J} \mathbf{\omega}) \right) \quad (\text{Euler's Rotational Dynamics})
\end{aligned}
$$


# Cascaded PID Controller for 6-DOF Quadrotor Dynamics

## Overview

A standard quadrotor has 4 control inputs (motor speeds $\omega_1, \omega_2, \omega_3, \omega_4$) to control 6 degrees of freedom (position $x, y, z$ and attitude $\phi, \theta, \psi$). Because it cannot produce direct lateral thrust, it is an **underactuated system**.

To map target trajectories to physical motor speeds, control is divided into **Cascaded Loops**:
1. **Outer Loop (Position PID):** Converts position errors into total thrust force $T$ and a target attitude quaternion $\mathbf{q}_{\text{des}}$.
2. **Inner Loop (Attitude PID):** Converts orientation errors into body torque commands $\boldsymbol{\tau} = [\tau_x, \tau_y, \tau_z]^T$.
3. **Control Allocation (Mixer Matrix):** Maps the unified control vector $\mathbf{u} = [T, \tau_x, \tau_y, \tau_z]^T$ to individual motor rotation speeds $\omega_i^2$.

---

## Step 1: Position PID $\rightarrow$ Total Thrust ($T$) & Desired Orientation ($\mathbf{q}_{\text{des}}$)

The outer loop tracks positional errors to generate horizontal acceleration (by tilting the body) and vertical thrust.

### 1. Position and Velocity Errors
$$\mathbf{e}_p = \mathbf{p}_{\text{des}} - \mathbf{p}, \quad \mathbf{e}_v = \mathbf{v}_{\text{des}} - \mathbf{v}$$

### 2. Desired Virtual Acceleration ($\mathbf{a}_{\text{des}}$)
Using a 3D PID control law:
$$\mathbf{a}_{\text{des}} = \mathbf{K}_p \mathbf{e}_p + \mathbf{K}_i \int \mathbf{e}_p \, dt + \mathbf{K}_d \mathbf{e}_v$$

### 3. Total Thrust Scalar ($T$)
The total desired force vector $\mathbf{f}_{\text{des}}$ compensates for gravity ($g$) and drone mass ($m$):
$$\mathbf{f}_{\text{des}} = m (\mathbf{a}_{\text{des}} + g \hat{\mathbf{z}})$$

The total thrust scalar $T$ is the magnitude of this force vector:
$$T = \|\mathbf{f}_{\text{des}}\|$$

### 4. Target Orientation Quaternion ($\mathbf{q}_{\text{des}}$)
The drone tilts its body $z$-axis along $\mathbf{f}_{\text{des}}$.

* **Desired Body Z-Axis:** $\mathbf{z}_b^{\text{des}} = \frac{\mathbf{f}_{\text{des}}}{\|\mathbf{f}_{\text{des}}\|}$
* **Desired Body X-Axis:** Projected using target yaw angle $\psi_{\text{des}}$:
  $$\mathbf{x}_b^{\text{des}} = \frac{\mathbf{y}_b^{\text{des}} \times \mathbf{z}_b^{\text{des}}}{\|\mathbf{y}_b^{\text{des}} \times \mathbf{z}_b^{\text{des}}\|}, \quad \text{where } \mathbf{y}_b^{\text{des}} = \mathbf{z}_b^{\text{des}} \times \begin{bmatrix} \cos\psi_{\text{des}} \\ \sin\psi_{\text{des}} \\ 0 \end{bmatrix}$$
* **Desired Rotation Matrix:** $\mathbf{R}_{\text{des}} = \begin{bmatrix} \mathbf{x}_b^{\text{des}} & \mathbf{y}_b^{\text{des}} & \mathbf{z}_b^{\text{des}} \end{bmatrix}$
* Convert $\mathbf{R}_{\text{des}}$ to unit quaternion representation $\mathbf{q}_{\text{des}}$.
---
## Step 2: Attitude PID to Body Torques (\tau)

The inner loop executes rapid rotational adjustments to align the current quaternion $\mathbf{q}$ with $\mathbf{q}_{\text{des}}$.

### 1. Quaternion Orientation Error ($\mathbf{q}_{\text{err}}$)

$$
\mathbf{q}_{\text{err}} = \mathbf{q}^{-1} \otimes \mathbf{q}_{\text{des}} = \begin{bmatrix} q_w \\ \mathbf{e}_q \end{bmatrix}
$$

*For small angle corrections, the vector component $\mathbf{e}_q = [e_{qx}, e_{qy}, e_{qz}]^T$ represents half of the body axis angle error.*

### 2. Angular Velocity Error ($\mathbf{e}_{\omega}$)

$$
\mathbf{e}_\omega = \mathbf{\omega}_{\text{des}} - \mathbf{\omega}
$$

### 3. Torque Vector Calculation ($\mathbf{\tau}$)

Applying PID control on rotational error outputs the target moment vector $\mathbf{\tau}$:

$$
\mathbf{\tau} = \mathbf{K}_{p,\theta} \mathbf{e}_q + \mathbf{K}_{i,\theta} \int \mathbf{e}_q \, dt + \mathbf{K}_{d,\theta} \mathbf{e}_\omega
$$
---
## Step 3: Mixer Matrix (Converting $\mathbf{u}$ to Motor Speeds $\omega_i^2$)

With $\mathbf{u} = \begin{bmatrix} T & \tau_x & \tau_y & \tau_z \end{bmatrix}^T$ determined, map the high-level inputs to physical motor forces using the quadrotor geometry parameters:
* $k_f$: Motor thrust coefficient
* $k_m$: Motor drag/torque coefficient
* $l$: Arm length from drone center to motor center

### Quad-X Mixer Matrix Allocation

$$
\begin{bmatrix} T \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix}
=
\begin{bmatrix}
k_f & k_f & k_f & k_f \\
-l \cdot k_f & l \cdot k_f & l \cdot k_f & -l \cdot k_f \\
l \cdot k_f & l \cdot k_f & -l \cdot k_f & -l \cdot k_f \\
-k_m & k_m & -k_m & k_m
\end{bmatrix}
\begin{bmatrix} \omega_1^2 \\ \omega_2^2 \\ \omega_3^2 \\ \omega_4^2 \end{bmatrix}
$$

### Solving for Motor Commands
To calculate individual squared motor angular velocities ($\omega_i^2$), invert the linear allocation matrix $\mathbf{M}$:

$$\begin{bmatrix} \omega_1^2 \\ \omega_2^2 \\ \omega_3^2 \\ \omega_4^2 \end{bmatrix} = \mathbf{M}^{-1} \begin{bmatrix} T \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix}$$

Finally, compute motor speeds $\omega_i = \sqrt{\max(0, \omega_i^2)}$ and map them to ESC PWM/DShot signal values.