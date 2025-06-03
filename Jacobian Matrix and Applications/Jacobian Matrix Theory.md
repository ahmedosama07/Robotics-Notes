### **6.1 Introduction to the Jacobian Matrix**
In robotics, the **Jacobian matrix** is a fundamental tool that relates joint velocities to end-effector velocities in space. Understanding and computing the Jacobian is crucial for velocity kinematics, force analysis, singularity detection, and control.

---
### **6.2 Definition and Mathematical Formulation**
Consider a robotic manipulator with joint variables $\mathbf{q} = [q_1, q_2, ..., q_n]^T$, where each $q_i$ can be a revolute (rotational) or prismatic (translational) joint parameter.
- The **end-effector pose** (position and orientation) is represented by a vector $\mathbf{x} = [x, y, z, \phi, \theta, \psi]^T$, where the first three components are translational coordinates, and the last three represent orientation (e.g., Euler angles or angular velocity vector components).
The **Jacobian matrix $\mathbf{J}(\mathbf{q})$** defines the linear mapping from joint velocities $\dot{\mathbf{q}}$ to end-effector velocity $\dot{\mathbf{x}}$:
$$
\dot{\mathbf{x}} = \mathbf{J}(\mathbf{q}) \dot{\mathbf{q}}
$$
where
$$
\dot{\mathbf{x}}
=
\begin{bmatrix} \mathbf{v} \\ \boldsymbol{\omega} \end{bmatrix}
=
\begin{bmatrix} v_x \\ v_y \\ v_z \\ \omega_x \\ \omega_y \\ \omega_z \end{bmatrix}
$$
---
### **6.3 Jacobian Composition for Revolute and Prismatic Joints**
For joint $i$, define:
- $\mathbf{z}_{i-1}$: the axis of motion of joint $i$ (unit vector), expressed in the base frame.
- $\mathbf{p}_n$: position vector of the end-effector (frame $n$) origin.
- $\mathbf{p}_{i-1}$: position vector of joint $i$ frame origin.
Then the $i$-th column of the Jacobian $\mathbf{J}$ is computed as:
- For a **revolute joint**:
$$
\mathbf{J}_i = \begin{bmatrix} \mathbf{z}_{i-1} \times (\mathbf{p}_n - \mathbf{p}_{i-1}) \\ \mathbf{z}_{i-1} \end{bmatrix}
$$
- For a **prismatic joint**:
$$
\mathbf{J}_i = \begin{bmatrix} \mathbf{z}_{i-1} \\ \mathbf{0} \end{bmatrix}
$$
---
### **6.4 Examples**
#### **Example 6.1: 3-DOF RRP Manipulator**
Recall the RRP manipulator (Revolute-Revolute-Prismatic joints):
- $q = [\theta_1, \theta_2, d_3]^T$
- $\mathbf{z}_0 = [0, 0, 1]^T$ (base frame z-axis)
- $\mathbf{z}_1$, $\mathbf{z}_2$ are computed using forward kinematics.
**Step 1:** Calculate positions $\mathbf{p}_n$, $\mathbf{p}_0$, $\mathbf{p}_1$, and $\mathbf{p}_2$ using forward kinematics.
**Step 2:** Calculate axes of motion $\mathbf{z}_0$, $\mathbf{z}_1$, $\mathbf{z}_2$.
**Step 3:** Compute Jacobian columns:
- For joint 1 (revolute):
$$
\mathbf{J}_1 = \begin{bmatrix} \mathbf{z}_0 \times (\mathbf{p}_n - \mathbf{p}_0) \\ \mathbf{z}_0 \end{bmatrix}
$$
- For joint 2 (revolute):
$$
\mathbf{J}_2 = \begin{bmatrix} \mathbf{z}_1 \times (\mathbf{p}_n - \mathbf{p}_1) \\ \mathbf{z}_1 \end{bmatrix}
$$
- For joint 3 (prismatic):
$$
\mathbf{J}_3 = \begin{bmatrix} \mathbf{z}_2 \\ \mathbf{0} \end{bmatrix}
$$
---
### **6.5 Velocity Kinematics**
The Jacobian relates joint velocities $\dot{\mathbf{q}}$ to end-effector linear and angular velocities:
$$
\dot{\mathbf{x}} = \mathbf{J}(\mathbf{q}) \dot{\mathbf{q}}
$$
This is crucial for:
- **Velocity control**
- **Trajectory tracking**
- **Singularity analysis**
---
### **6.6 Inverse Velocity Kinematics**
If $\mathbf{J}$ is square and nonsingular, the inverse velocity can be found by:
$$
\dot{\mathbf{q}} = \mathbf{J}^{-1}(\mathbf{q}) \dot{\mathbf{x}}
$$
For redundant or under-actuated systems, use the pseudoinverse $\mathbf{J}^+$ or null-space techniques.

---
### **6.7 Singularities: When Does the Jacobian Lose Rank?**
A singularity occurs when:
$$
\det(\mathbf{J}) = 0
$$
At singularities:
- The manipulator loses some DOF
- The inverse Jacobian does not exist
- Infinite joint velocities may be required for some end-effector velocities
---
### **6.8 Force/Torque Relationship: Jacobian Transpose**
The Jacobian transpose relates **joint torques $\tau$** to **end-effector forces $\mathbf{F}$**:
$$
\tau = \mathbf{J}^T \mathbf{F}
$$
where:
- $\mathbf{F} \in \mathbb{R}^6$ includes forces and moments at the end-effector
- $\tau \in \mathbb{R}^n$ is the vector of joint torques/forces
---
### **6.9 Summary Box**
$$
\boxed{ \begin{aligned} & \dot{\mathbf{x}} = \mathbf{J}(\mathbf{q}) \dot{\mathbf{q}} \\ & \mathbf{J}_i = \begin{cases} \begin{bmatrix} \mathbf{z}_{i-1} \times (\mathbf{p}_n - \mathbf{p}_{i-1}) \\ \mathbf{z}_{i-1} \end{bmatrix} & \text{revolute joint}\\ \begin{bmatrix} \mathbf{z}_{i-1} \\ \mathbf{0} \end{bmatrix} & \text{prismatic joint} \end{cases} \\ & \tau = \mathbf{J}^T \mathbf{F} \end{aligned} }
$$
---
### **6.10 Jacobian for a 3-DOF RRP Manipulator**
**Robot Description Recap:**
- **Joint 1:** Revolute about $z_0 = [0,0,1]^T$
- **Joint 2:** Revolute about $z_1$ (rotated frame)
- **Joint 3:** Prismatic along $z_2$ (link extension)

**Step 1: Forward Kinematics to find positions and axes**
Let’s define:
- Base frame origin $O_0 = [0,0,0]^T$
- $O_1$: after first joint rotation
- $O_2$: after second joint rotation
- End-effector position $P = O_3$
Using Denavit-Hartenberg parameters:

| Joint | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
| ----- | ---------- | ----- | ----- | ---------- |
| 1     | $\theta_1$ | 0     | 0     | $\pi/2π$   |
| 2     | $\theta_2$ | 0     | $a_2$ | 0          |
| 3     | 0          | d_3$  | 0     | 0          |

---
**Step 2: Compute transformation matrices $T_1$, $T_2$, $T_3$**
$$
T_1 = \begin{bmatrix} \cos \theta_1 & -\sin \theta_1 & 0 & 0 \\ \sin \theta_1 & \cos \theta_1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
$$
T_2 = \begin{bmatrix} \cos \theta_2 & -\sin \theta_2 & 0 & a_2 \cos \theta_2 \\ \sin \theta_2 & \cos \theta_2 & 0 & a_2 \sin \theta_2 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$$$
T_3 = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & d_3 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
---
**Step 3: Compute overall transformation $T = T_1 T_2 T_3$**
$$
T = T_1 \times T_2 \times T_3
$$
Extract end-effector position $\mathbf{p}_n$ from $T$:
$$
\mathbf{p}_n = \begin{bmatrix} x \\ y \\ z \end{bmatrix}
$$
---
**Step 4: Extract joint axes $\mathbf{z}_0$, $\mathbf{z}_1$, $\mathbf{z}_2$**
- $\mathbf{z}_0 = [0, 0, 1]^T$
- $\mathbf{z}_1 = T_1$ rotation axis z (third column)
- $\mathbf{z}_2 = T_1 T_2$ rotation axis z (third column)
---
**Step 5: Compute positions $\mathbf{p}_0$, $\mathbf{p}_1$, $\mathbf{p}_2$**
- $\mathbf{p}_0 = [0,0,0]^T$
- $\mathbf{p}_1$ from $T_1$
- $\mathbf{p}_2$​ from $T_1$ $T_2$
---
**Step 6: Compute Jacobian columns**
$$
J_i = \begin{cases}
\begin{bmatrix} \mathbf{z}_{i-1} \times (\mathbf{p}_n - \mathbf{p}_{i-1}) \\ \mathbf{z}_{i-1} \end{bmatrix} & \text{revolute joint} \\
\begin{bmatrix} \mathbf{z}_{i-1} \\ \mathbf{0} \end{bmatrix} & \text{prismatic joint}
\end{cases}
$$
---
### **6.11 Velocity Propagation Through the Kinematic Chain**
The velocity of each link frame iii can be recursively computed as:
$$
\boldsymbol{\omega}_i = \mathbf{R}_{i}^{i-1} \boldsymbol{\omega}_{i-1} + \dot{q}_i \mathbf{z}_i
$$$$\mathbf{v}_i = \mathbf{R}_{i}^{i-1} \left( \mathbf{v}_{i-1} + \boldsymbol{\omega}_{i-1} \times \mathbf{p}_{i}^{i-1} \right) + \dot{d}_i \mathbf{z}_i
$$
Where
- $\mathbf{R}_{i}^{i-1}$ is rotation matrix from frame $i-1$ to $i$,
- $\mathbf{p}_{i}^{i-1}$ is position vector from $i-1$ to $i$,
- $\dot{q}_i$ is joint rate (for revolute), and
- $\dot{d}_i$ is joint linear velocity (for prismatic).
---
### **6.12 Singularity Analysis for the RRP Manipulator**

**Singular Configurations**
- Occur when $\det(J) = 0$
- At singularities, manipulator loses one or more DOF.
- For example, when $\theta_2 = 0$ or $\pi$, the arm is fully stretched or folded, a typical singularity.

**Effect on Manipulator Performance**
- Near singularities, small end-effector velocities require large joint velocities.
- Control becomes unstable.
- Torque requirements become very large.
---
