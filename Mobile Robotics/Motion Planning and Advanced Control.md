### **10.1 Introduction**
- Overview of motion planning as **path generation** and **trajectory generation**.
- Importance of considering **kinematics**, **dynamics**, and **environmental constraints**.
- High-level **motion planners** (e.g., sampling-based) vs. **trajectory optimizers**.
---
#### **10.2 Sampling-Based Motion Planners**
- Random sampling: covers complex, high-dimensional configuration spaces.
- Examples:
    - **Probabilistic Roadmap (PRM)**: good for multiple queries in static environments.
    - **Rapidly-exploring Random Tree (RRT)**: fast single-query planners.
- Extensions:
    - **RRT***: asymptotically optimal.
    - **PRM***: better for connectedness and optimality.
---
### **10.3 Trajectory Generation and Optimization**
- **Trajectory generation**: defines $\mathbf{q}(t)$ or $\mathbf{x}(t)$ (task-space).
- **Trajectory optimization**:
    - Minimum-time or minimum-jerk profiles.
    - Constrained optimization (e.g., joint limits, torque limits).
    - Use of **splines (e.g., B-splines, cubic)** for smoothness.
---
### **10.4 Advanced Controllers**
- Beyond PID:
    - **Computed Torque Control (CTC)**: uses dynamic model to cancel nonlinearities.
    - **Adaptive Control**: adjusts to parameter uncertainties.
    - **Impedance/Admittance Control**: for interaction with environment (force control).
---
### **10.5 Dynamic Obstacle Avoidance**
- Dynamic environments require **real-time replanning**.
- Methods:
    - Velocity obstacles.
    - Dynamic Window Approach (DWA).
    - Sampling-based planners with dynamic updates.
---
### **10.6 Integration with Perception Systems**
- Use of **vision/LiDAR** for obstacle/environment mapping.
- Realtime **SLAM (Simultaneous Localization and Mapping)**.
- **Reactive control** integrated with high-level planning.
---
### **10.7 Discussion: Comparison of Motion Planning Approaches**
- **PRM**: better for static environments & repeated queries.
- **RRT**: faster for single-query tasks, dynamic updates easier.
- **Optimization-based**: more computationally intensive but higher quality (e.g., CHOMP, TrajOpt).
- **Learning-based planners**: promising for real-time adaptation.