### **11.1 Introduction to Human-Robot Interaction (HRI)**
Human-Robot Interaction (HRI) studies the dynamics between humans and robots operating in shared environments. The goal is to enable robots to safely, efficiently, and intuitively collaborate with humans.
**Key aspects:**
- Safety: Avoid collisions and harm.
- Efficiency: Maximize joint productivity.
- Intuitiveness: Easy human understanding and robot behavior prediction.
- Adaptability: React and adapt to human actions.
---
### **11.2 Modeling Human-Robot Systems**
We model a human-robot collaborative system as a **coupled dynamic system**:
$$
\mathbf{M}_r(\mathbf{q}) \ddot{\mathbf{q}} + \mathbf{C}_r(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}} + \mathbf{G}_r(\mathbf{q}) = \boldsymbol{\tau}_r + \boldsymbol{\tau}_h
$$
- $\mathbf{q} \in \mathbb{R}^n$: robot joint coordinates (DOF = $n$)
- $\boldsymbol{\tau}_r$: robot actuation torques
- $\boldsymbol{\tau}_h$: human-applied interaction torques
- $\mathbf{M}_r$: robot inertia matrix
- $\mathbf{C}_r$: Coriolis/centrifugal matrix
- $\mathbf{G}_r$: gravity vector
Similarly, human dynamics:
$$
\mathbf{M}_h(\mathbf{q}_h) \ddot{\mathbf{q}}_h + \mathbf{C}_h(\mathbf{q}_h, \dot{\mathbf{q}}_h) \dot{\mathbf{q}}_h + \mathbf{G}_h(\mathbf{q}_h) = \boldsymbol{\tau}_h^{int} + \boldsymbol{\tau}_r^{int}
$$
- $\mathbf{q}_h$​: human joint coordinates
- $\boldsymbol{\tau}_h^{int}$​: human muscle torques
- $\boldsymbol{\tau}_r^{int}$: torque from robot interaction
---
### **11.3 Safety in Physical Human-Robot Interaction (pHRI)**
#### **11.3.1 Collision Avoidance**
Robots must detect and avoid collisions with humans. Given:
- Robot surface points $\mathbf{p}_r$
- Human body points $\mathbf{p}_h$
Define the **signed distance function** $d(\mathbf{p}_r, \mathbf{p}_h)$ that returns the shortest distance, negative if penetration occurs.
**Collision condition:**
$$
d(\mathbf{p}_r, \mathbf{p}_h) \leq d_{safe}
$$
where $d_{safe}$ is a safety margin.

---
#### **11.3.2 Safety Control via Potential Fields**
We define an artificial potential:
$$
U(\mathbf{p}_r, \mathbf{p}_h) = 
\begin{cases}
\frac{1}{2} k_{rep} \left( \frac{1}{d(\mathbf{p}_r, \mathbf{p}_h)} - \frac{1}{d_{safe}} \right)^2 & d \leq d_{safe} \\
0 & d > d_{safe}
\end{cases}
$$
where $k_{rep} > 0$ is a repulsive gain.
Force applied on robot:
$$
\mathbf{F}_{rep} = -\nabla_{\mathbf{p}_r} U(\mathbf{p}_r, \mathbf{p}_h)
$$
This force is fed into the control law to push the robot away from the human when they come too close.

---
### **11.4 Control Architectures for HRI**
#### **11.4.1 Impedance Control**
Impedance control models the robot end-effector as a dynamic system with mass, damping, and stiffness that interacts compliantly with the human:
$$
\mathbf{M}_d (\ddot{\mathbf{x}} - \ddot{\mathbf{x}}_d) + \mathbf{B}_d (\dot{\mathbf{x}} - \dot{\mathbf{x}}_d) + \mathbf{K}_d (\mathbf{x} - \mathbf{x}_d) = \mathbf{F}_{ext}
$$
- $\mathbf{x}$, $\dot{\mathbf{x}}$, $\ddot{\mathbf{x}}$: actual end-effector pose, velocity, acceleration
- $\mathbf{x}_d$, $\dot{\mathbf{x}}_d$, $\ddot{\mathbf{x}}_d$: desired trajectory
- $\mathbf{M}_d$, $\mathbf{B}_d$, $\mathbf{K}_d$: desired inertia, damping, stiffness matrices
- $\mathbf{F}_{ext}$: external force applied by human
This enables the robot to behave like a virtual mass-spring-damper system reacting to human forces.
---
#### **11.4.2 Admittance Control**
Admittance control takes the measured external force and outputs desired motion:
$$
\mathbf{M}_a \ddot{\mathbf{x}} + \mathbf{B}_a \dot{\mathbf{x}} + \mathbf{K}_a \mathbf{x} = \mathbf{F}_{ext}
$$
Parameters can be tuned for desired compliance and responsiveness.

---
#### **11.4.3 Hybrid Position/Force Control**
In some tasks, position control is needed in some directions and force control in others. The task space is decomposed into orthogonal subspaces:
$$
\mathbf{u} = \mathbf{P} \mathbf{u}_p + \mathbf{F} \mathbf{u}_f
$$
where $\mathbf{P}$, $\mathbf{F}$ are projection matrices onto position and force controlled subspaces respectively, with $\mathbf{P} + \mathbf{F} = \mathbf{I}$.

---
### **11.5 Human Intent Recognition**
Robots can predict human intention using sensor data and models.
#### **11.5.1 Probabilistic Models**
Using **Hidden Markov Models (HMMs)** or **Bayesian Networks**:
- States: Human intentions/tasks
- Observations: Sensor data (e.g., motion capture, EMG)
Estimate most likely intention $I^*$:
$$
I^* = \arg \max_{I} P(I \mid \text{observations})
$$
---
#### **11.5.2 Machine Learning Approaches**
Use supervised learning on labeled interaction data to classify human intent.

---
### **11.6 Collaborative Task Allocation and Shared Autonomy**
#### **11.6.1 Optimization of Task Allocation**
Minimize a cost function $J$ over human and robot effort:
$$
J = w_h E_h + w_r E_r + w_s E_s
$$
- $E_h$, $E_r$: energy or effort for human and robot
- $E_s$: safety or social cost
- $w_h$, $w_r$, $w_s$: weights
Subject to:
- Task constraints
- Human/robot capabilities
---
#### **11.6.2 Shared Autonomy**
Control blending between human input $\mathbf{u}_h$ and robot autonomy $\mathbf{u}_r$:
$$
\mathbf{u} = \alpha \mathbf{u}_h + (1-\alpha) \mathbf{u}_r
$$
where $\alpha \in [0,1]$ varies adaptively based on context and confidence.

---
### **11.7 Communication Modalities**
Robots interpret:
- Voice commands (natural language processing)
- Gestures (computer vision, sensor fusion)
- Physical interaction (force/torque sensors)
Mathematical models include:
- Hidden Markov Models for gesture recognition
- NLP probabilistic parsing
- Sensor fusion filters (Kalman, particle filters)
---
### **11.8 Case Studies**
#### **11.8.1 Collaborative Assembly**
- Model interaction forces during cooperative part handling
- Use impedance control to modulate compliance during human-guided motion
- Evaluate stability margins and passivity conditions to ensure safe interaction
#### **11.8.2 Assistive Robots for Rehabilitation**
- Adapt robot stiffness and damping parameters to patient’s strength
- Implement adaptive impedance control with real-time parameter identification
---
### **11.9 Summary**
- Human-robot collaboration requires rigorous safety, control, and communication strategies.
- Dynamic models integrating human and robot enable predictive and adaptive control.
- Probabilistic methods help in understanding and predicting human intent.
- Advanced control schemes (impedance, admittance, hybrid) provide compliant interaction.
- Task allocation and shared autonomy optimize cooperation efficiency and safety.