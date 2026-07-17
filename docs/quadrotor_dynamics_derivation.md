# Quadrotor Dynamics Model Derivation

## 1. System Definition and Coordinate Frames

### Coordinate Systems
- **Inertial Frame (World Frame) {W}**: Fixed reference frame with axes X, Y, Z
  - Z-axis points upward (opposite to gravity)
  - Origin at arbitrary fixed point
  - Position: **p** = [x, y, z]ᵀ

- **Body-Fixed Frame {B}**: Attached to the quadrotor body with axes x_b, y_b, z_b
  - Origin at the center of mass
  - z_b points upward from the quadrotor body
  - Rotates with the quadrotor

### Rotation Matrix
The rotation from body frame to inertial frame is represented by the rotation matrix R ∈ SO(3):

$$\mathbf{R} = \begin{bmatrix} 
c\theta c\psi & s\phi s\theta c\psi - c\phi s\psi & c\phi s\theta c\psi + s\phi s\psi \\
c\theta s\psi & s\phi s\theta s\psi + c\phi c\psi & c\phi s\theta s\psi - s\phi c\psi \\
-s\theta & s\phi c\theta & c\phi c\theta
\end{bmatrix}$$

Where:
- φ (phi): roll angle - rotation about x-axis
- θ (theta): pitch angle - rotation about y-axis  
- ψ (psi): yaw angle - rotation about z-axis
- c(·) = cos(·), s(·) = sin(·)

### State Variables
- **Position**: **p** = [x, y, z]ᵀ (inertial frame)
- **Velocity**: **v** = [ẋ, ẏ, ż]ᵀ (inertial frame)
- **Orientation**: Euler angles **η** = [φ, θ, ψ]ᵀ
- **Angular velocity**: **ω** = [p, q, r]ᵀ (body frame), where:
  - p = φ̇ - ψ̇ sin(θ)
  - q = θ̇ cos(φ) + ψ̇ sin(φ) cos(θ)
  - r = -θ̇ sin(φ) + ψ̇ cos(φ) cos(θ)

---

## 2. Forces Acting on the Quadrotor

### Gravity Force
$$\mathbf{F}_g = \begin{bmatrix} 0 \\ 0 \\ -mg \end{bmatrix}$$

where m is the total mass and g is gravitational acceleration.

### Thrust Force
The four motors generate thrust forces. Each motor i produces a thrust force:
$$f_i = b \omega_i^2$$

where:
- ω_i is the angular velocity of motor i
- b is the thrust coefficient

The total thrust in the body z-direction is:
$$f_z^b = \sum_{i=1}^{4} f_i = b(\omega_1^2 + \omega_2^2 + \omega_3^2 + \omega_4^2)$$

This thrust acts only along the body z-axis (upward direction of the quadrotor).

### Total Force in Body Frame
$$\mathbf{F}^b = \begin{bmatrix} 0 \\ 0 \\ f_z^b \end{bmatrix}$$

### Total Force in Inertial Frame
$$\mathbf{F}^W = \mathbf{R} \mathbf{F}^b = f_z^b \mathbf{R} \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$

where $\mathbf{R} \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$ is the third column of R, representing the z-axis of the body frame in the inertial frame.

---

## 3. Torques Acting on the Quadrotor

### Motor Torque and Drag
Each motor produces a torque about its axis due to the drag force:
$$\tau_i = d \omega_i^2$$

where d is the drag coefficient.

The drag torque from motor i acts about the z-axis of the quadrotor and causes yaw rotation. For a typical X-configuration quadrotor:
- Motors 1 and 3 rotate counter-clockwise (viewed from top)
- Motors 2 and 4 rotate clockwise

### Total Yaw Torque
$$\tau_z = d(\omega_1^2 - \omega_2^2 + \omega_3^2 - \omega_4^2)$$

or equivalently:
$$\tau_z = d \sum_{i=1}^{4} (-1)^{i+1} \omega_i^2$$

### Rolling and Pitching Torques
The thrust difference between motors creates roll and pitch moments. Let l be the distance from the center of mass to each motor:

**Roll torque** (rotation about x-axis):
$$\tau_x = l \cdot b(\omega_3^2 - \omega_1^2)$$

**Pitch torque** (rotation about y-axis):
$$\tau_y = l \cdot b(\omega_2^2 - \omega_4^2)$$

Actually, for a more standard configuration:
$$\tau_x = l \cdot b(\omega_4^2 - \omega_2^2)$$
$$\tau_y = l \cdot b(\omega_1^2 - \omega_3^2)$$

### Summary - Torque Vector in Body Frame
$$\boldsymbol{\tau}^b = \begin{bmatrix} 
l \cdot b(\omega_4^2 - \omega_2^2) \\
l \cdot b(\omega_1^2 - \omega_3^2) \\
d(\omega_1^2 - \omega_2^2 + \omega_3^2 - \omega_4^2)
\end{bmatrix}$$

Or in matrix form:
$$\boldsymbol{\tau}^b = \begin{bmatrix} 
0 & -lb & 0 & lb \\
lb & 0 & -lb & 0 \\
d & -d & d & -d
\end{bmatrix} \begin{bmatrix} \omega_1^2 \\ \omega_2^2 \\ \omega_3^2 \\ \omega_4^2 \end{bmatrix}$$

---

## 4. Translational Dynamics (Newton's Second Law)

Newton's second law in the inertial frame:
$$m\ddot{\mathbf{p}} = \mathbf{F}^W + \mathbf{F}_g$$

$$m\ddot{\mathbf{p}} = f_z^b \mathbf{R} \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} + \begin{bmatrix} 0 \\ 0 \\ -mg \end{bmatrix}$$

Let **z_b** = $\mathbf{R} \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$ be the unit vector along the body z-axis in the inertial frame.

Expanding with Euler angles:
$$z_b = \begin{bmatrix} -\sin\theta \\ \sin\phi\cos\theta \\ \cos\phi\cos\theta \end{bmatrix}$$

### Translational Equations of Motion
$$\ddot{x} = \frac{f_z^b}{m}(-\sin\theta \cos\psi - \sin\phi \cos\phi \sin\psi)$$

$$\ddot{y} = \frac{f_z^b}{m}(-\sin\theta \sin\psi + \sin\phi \cos\phi \cos\psi)$$

$$\ddot{z} = -g + \frac{f_z^b}{m}\cos\phi\cos\theta$$

Or more compactly:
$$\ddot{\mathbf{p}} = g\mathbf{e}_z + \frac{f_z^b}{m}\mathbf{R}\mathbf{e}_3$$

where **e_z** = [0, 0, 1]ᵀ and **e_3** = [0, 0, 1]ᵀ

---

## 5. Rotational Dynamics (Euler's Equation)

The equation for rotational motion using the inertia tensor **I** is:
$$\mathbf{I}\dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times \mathbf{I}\boldsymbol{\omega} = \boldsymbol{\tau}^b$$

For a quadrotor with symmetric mass distribution (diagonal inertia tensor):
$$\mathbf{I} = \begin{bmatrix} I_x & 0 & 0 \\ 0 & I_y & 0 \\ 0 & 0 & I_z \end{bmatrix}$$

Assuming I_x ≈ I_y ≈ I (symmetric quadrotor):
$$\mathbf{I} = \begin{bmatrix} I & 0 & 0 \\ 0 & I & 0 \\ 0 & 0 & I_z \end{bmatrix}$$

### Rotational Equations Component-wise

**Roll dynamics:**
$$I\dot{p} + (I_z - I)qr = \tau_x$$

**Pitch dynamics:**
$$I\dot{q} + (I - I_z)pr = \tau_y$$

**Yaw dynamics:**
$$I_z\dot{r} = \tau_z$$

In expanded form with control inputs:
$$\dot{p} = \frac{1}{I}[\tau_x + (I_z - I)qr]$$

$$\dot{q} = \frac{1}{I}[\tau_y + (I - I_z)pr]$$

$$\dot{r} = \frac{1}{I_z}\tau_z$$

### Relating Angular Velocity to Euler Angle Rates

The relationship between body angular velocities (p, q, r) and Euler angle rates:
$$\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} 
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi/\cos\theta & \cos\phi/\cos\theta
\end{bmatrix} \begin{bmatrix} p \\ q \\ r \end{bmatrix}$$

Or inverted:
$$\begin{bmatrix} p \\ q \\ r \end{bmatrix} = \begin{bmatrix} 
1 & 0 & -\sin\theta \\
0 & \cos\phi & \sin\phi\cos\theta \\
0 & -\sin\phi & \cos\phi\cos\theta
\end{bmatrix} \begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix}$$

---

## 6. Motor Input Mapping

The four motor commands ω₁², ω₂², ω₃², ω₄² map to the control inputs:

$$\begin{bmatrix} f_z^b \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix} = \begin{bmatrix} 
b & b & b & b \\
0 & -lb & 0 & lb \\
lb & 0 & -lb & 0 \\
d & -d & d & -d
\end{bmatrix} \begin{bmatrix} \omega_1^2 \\ \omega_2^2 \\ \omega_3^2 \\ \omega_4^2 \end{bmatrix}$$

Let **u** = [u₁, u₂, u₃, u₄]ᵀ = [ω₁², ω₂², ω₃², ω₄²]ᵀ

Then:
$$u_1 = b(u_1 + u_2 + u_3 + u_4) = f_z^b$$

$$u_2 = -lb u_2 + lb u_4 = \tau_x$$

$$u_3 = lb u_1 - lb u_3 = \tau_y$$

$$u_4 = d(u_1 - u_2 + u_3 - u_4) = \tau_z$$

---

## 7. Complete System Dynamics Summary

### State Vector
$$\mathbf{x} = [x, y, z, \dot{x}, \dot{y}, \dot{z}, \phi, \theta, \psi, p, q, r]^T$$

### State Equations - Position Dynamics
$$\begin{align}
\dot{x} &= v_x \\
\dot{y} &= v_y \\
\dot{z} &= v_z
\end{align}$$

### State Equations - Velocity Dynamics
$$\begin{align}
\ddot{x} &= \frac{u_1}{m}(\sin\psi\sin\phi + \cos\psi\sin\theta\cos\phi) \\
\ddot{y} &= \frac{u_1}{m}(\sin\psi\sin\theta\cos\phi - \cos\psi\sin\phi) \\
\ddot{z} &= -g + \frac{u_1}{m}\cos\theta\cos\phi
\end{align}$$

### State Equations - Attitude Dynamics
$$\begin{align}
\dot{\phi} &= p + q\sin\phi\tan\theta + r\cos\phi\tan\theta \\
\dot{\theta} &= q\cos\phi - r\sin\phi \\
\dot{\psi} &= q\sin\phi/\cos\theta + r\cos\phi/\cos\theta
\end{align}$$

### State Equations - Angular Rate Dynamics
$$\begin{align}
\dot{p} &= \frac{1}{I_x}[u_2 + (I_y - I_z)qr] \\
\dot{q} &= \frac{1}{I_y}[u_3 + (I_z - I_x)pr] \\
\dot{r} &= \frac{1}{I_z}u_4
\end{align}$$

Where:
- m: total mass
- g: gravitational acceleration (≈ 9.81 m/s²)
- Ix, Iy, Iz: moments of inertia
- b: thrust coefficient
- l: distance from center of mass to motor
- d: drag coefficient
- u₁: total thrust (N)
- u₂, u₃, u₄: roll, pitch, yaw torques (N·m)

---

## 8. Simplifications and Assumptions

The derivation above assumes:

1. **Rigid body**: The quadrotor frame is rigid with no flexibility
2. **Symmetric mass distribution**: Inertia tensor is diagonal with I_x ≈ I_y
3. **Negligible aerodynamic drag**: Except for motor drag
4. **Linear thrust model**: f = bω²
5. **Small Euler angles**: Not strictly assumed in full model, but often used in linearized versions
6. **No blade flapping**: Fixed-pitch propellers
7. **Quasi-static motor response**: Motors respond instantaneously to speed commands

---

## 9. Alternative Forms

### Compact Vector Form
$$m\ddot{\mathbf{p}} = -mg\mathbf{e}_z + u_1 R\mathbf{e}_3$$

$$\mathbf{I}\dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times \mathbf{I}\boldsymbol{\omega} = \boldsymbol{\tau}$$

### Linearized Model (Small Angle Approximation)
For small angles (φ, θ << 1) and near hover condition where u₁ ≈ mg:

$$\ddot{x} \approx g\theta$$
$$\ddot{y} \approx -g\phi$$
$$\ddot{z} \approx \frac{u_1 - mg}{m}$$
$$\ddot{\phi} \approx \frac{u_2}{I}$$
$$\ddot{\theta} \approx \frac{u_3}{I}$$
$$\ddot{\psi} \approx \frac{u_4}{I_z}$$

This linearized model is much simpler for control design and stability analysis near the hover condition.

---

## References

- Beard, R. W., & McLain, T. W. (2012). Small Unmanned Aircraft: Theory and Practice
- Bouabdallah, S. (2007). Design and Control of Quadrotors with Application to Autonomous Flying
- Mellinger, D., & Kumar, V. (2011). Minimum snap trajectory generation and control for quadrotors
- Valavanis, K. P., & Vachtsevanos, G. J. (Eds.). (2015). Handbook of Unmanned Aerial Vehicles
