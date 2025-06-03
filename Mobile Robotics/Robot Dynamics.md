### **9.1 Introduction to Dynamics**
- Kinematics vs. Dynamics:
    - Kinematics → Motion without considering forces.
    - Dynamics → Motion **with** forces, torques, and mass.
- Two approaches:
    - **Forward dynamics**: Given torques/forces → find motion.
    - **Inverse dynamics**: Given desired motion → find required torques/forces.
---
### **9.2 Equations of Motion**
- For a manipulator with $n$ DOF:
  $$
\mathbf{M}(\mathbf{q}) \ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau}
$$    where:
    - $\mathbf{q}$: joint positions
    - $\dot{\mathbf{q}}$​: joint velocities
    - $\ddot{\mathbf{q}}$: joint accelerations
    - $\mathbf{M}(\mathbf{q})$: inertia matrix
    - $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})$: Coriolis and centrifugal effects
    - $\mathbf{g}(\mathbf{q})$: gravity torques
    - $\boldsymbol{\tau}$: joint torques
---
### **8.3 Derivation Methods**
- **Newton-Euler** (recursive, efficient)
- **Lagrangian** (energy-based, systematic)
---
### **8.4 Example: 2-Link Planar Manipulator (RR Arm)**
1. Kinetic energy:
$$
T = \frac{1}{2} \dot{\mathbf{q}}^T \mathbf{M}(\mathbf{q}) \dot{\mathbf{q}}
$$
2. Potential energy:
$$
V = m_1 g h_1 + m_2 g h_2
$$
3. Lagrangian:
$$
\mathcal{L} = T - V
$$
4. Euler-Lagrange equations:
$$
\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}_i} \right) - \frac{\partial \mathcal{L}}{\partial q_i} = \tau_i
$$
---
### **8.5 Inverse Dynamics**
- Compute $\boldsymbol{\tau}$ for a given trajectory $\mathbf{q}(t)$.
- Used in feedforward control & motion planning.
---
### **8.6 Forward Dynamics**
- Compute $\ddot{\mathbf{q}}$ for a given torque $\boldsymbol{\tau}$.
- Simulation of robot motion under external forces.