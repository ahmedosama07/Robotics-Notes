### **8.1 Introduction**
Trajectory planning and control is about **generating paths** and **control signals** for a robot manipulator to move its end-effector from an initial pose to a desired final pose over time, while respecting:
- Joint limits
- Velocity and acceleration constraints
- Dynamic capabilities
- Avoiding obstacles
---
### **8.2 Types of Trajectories**
1. **Point-to-Point (PTP) Trajectories:**
    - Move from start to end pose without specifying intermediate points.
    - Commonly use polynomial interpolation (e.g., cubic, quintic polynomials).
    - Smooth position, velocity, and acceleration profiles.
2. **Continuous Path Trajectories:**
    - Robot must pass through a sequence of via points.
    - Used in welding, painting, machining.
3. **Joint Space vs Task Space Trajectories:**
    - **Joint space:** trajectories planned for each joint angle.
    - **Task space:** trajectories planned in Cartesian space (end-effector position/orientation).
    - Task space planning usually requires inverse kinematics.
---
### **8.3 Trajectory Generation Methods**
#### **8.3.1 Polynomial Trajectories**
- **Cubic Polynomial:**
$$
\theta(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3
$$
Boundary conditions typically specify initial and final position and velocity:
$$
\theta(0) = \theta_0, \quad \theta(t_f) = \theta_f
$$
$$
\dot{\theta}(0) = \dot{\theta}_0, \quad \dot{\theta}(t_f) = \dot{\theta}_f
$$
- **Quintic Polynomial:**
Adds acceleration boundary conditions for smoother motion.
#### **8.3.2 Trapezoidal Velocity Profiles**
- Accelerate at constant rate, cruise at constant velocity, then decelerate.
- Common for simple motion control.
---
### **8.4 Inverse Kinematics in Trajectory Planning**
- When trajectory is specified in Cartesian space, inverse kinematics must be solved at each time step to find joint positions.
- Requires careful handling of multiple solutions and singularities.
---
### **8.5 Feedback Control for Trajectory Tracking**
- Open-loop control (simply feeding joint commands) is sensitive to model errors and disturbances.
- Use feedback controllers to correct errors in position and velocity.
#### **8.5.1 PD Control in Joint Space**
$$
\tau = K_p (\mathbf{q}_d - \mathbf{q}) + K_d (\dot{\mathbf{q}}_d - \dot{\mathbf{q}})
$$
- $\mathbf{q}_d$, $\dot{\mathbf{q}}_d$: desired joint positions and velocities
- $K_p$, $K_d$: proportional and derivative gain matrices
#### **8.5.2 Computed Torque Control (Inverse Dynamics Control)**
Uses robot dynamics model:
$$
\tau = M(\mathbf{q}) \ddot{\mathbf{q}}_d + C(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}}_d + G(\mathbf{q}) + K_p \mathbf{e} + K_d \dot{\mathbf{e}}
$$
Where
- $M$: inertia matrix
- $C$: Coriolis/centrifugal matrix
- $G$: gravity vector
- $\mathbf{e} = \mathbf{q}_d - \mathbf{q}$
---
### **8.6 Example: Cubic Polynomial Trajectory for a Single Joint**
Given
- Start angle $\theta_0 = 0\degree$
- Final angle $\theta_f = 90\degree$
- Duration $t_f = 5$ seconds
- Start and end velocities zero
Find polynomial coefficients:
$$
\begin{cases} \theta(0) = a_0 = \theta_0 \\ \theta(t_f) = a_0 + a_1 t_f + a_2 t_f^2 + a_3 t_f^3 = \theta_f \\ \dot{\theta}(0) = a_1 = 0 \\ \dot{\theta}(t_f) = a_1 + 2 a_2 t_f + 3 a_3 t_f^2 = 0 \end{cases}
$$
---
### **8.7 Trajectory Planning for Multi-DOF Manipulators**
- Plan trajectories for each joint independently or coordinated in task space.
- Ensure joint limits and velocity constraints are respected.
- Use inverse kinematics and dynamics models to validate feasibility.