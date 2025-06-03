### **1.1 Definition of a Robot**

A **robot** is an electromechanical device that can perform tasks autonomously or semi-autonomously, programmed or controlled by humans. A robot typically:

- Has mechanical components (links, joints, actuators).
- Uses sensors to interact with the environment.
- Processes data via a controller or onboard computer.
- Adapts to changes in its environment.

**Key Attributes:**
$$
\boxed{ \begin{aligned} &\text{Reprogrammable} \\ &\text{Multifunctional} \\ &\text{Environmentally aware via sensors} \\ &\text{Autonomous or semi-autonomous} \\ \end{aligned} }
$$
---

### **1.2 Isaac Asimov’s Three Laws of Robotics**

1️⃣ **First Law:**
A robot may not injure a human being or, through inaction, allow a human being to come to harm
$$
\boxed{\text{A robot may not injure a human being or, through inaction, allow a human being to come to harm}}
$$

2️⃣ **Second Law:**
$$
\boxed{\text{A robot must obey the orders given it by human beings except where such orders would conflict with the First Law}}
$$
3️⃣ **Third Law:**
$$
\boxed{\text{A robot must protect its own existence as long as such protection does not conflict with the First or Second Law}}
$$

---

### **1.3 Robot Classifications**

#### **1.3.1 Japanese Industrial Robot Association (JIRA)**
- **Class 1:** Manual manipulators
- **Class 2:** Fixed sequence robots
- **Class 3:** Variable sequence robots
- **Class 4:** Playback robots
- **Class 5:** Numerically controlled robots
- **Class 6:** Intelligent robots
#### **1.3.2 Robotics Institute of America (RIA)**
- **Class 3:** Playback robots
- **Class 4:** Numerically controlled robots
- **Class 5:** Intelligent robots
- **Class 6:** Adaptive/intelligent robots
#### **1.3.3 Association Francaise de Robotique (AFR)**
- **Type A:** Manipulators
- **Type B:** Sequence-controlled robots
- **Type C:** Sensor-controlled robots
- **Type D:** Adaptive robots
---
### **1.4 Robot Components**

|Component|Description|
|---|---|
|Manipulator|Mechanical arm with links and joints|
|End Effector|Device at the end of manipulator (e.g., gripper, tool)|
|Actuators|Motors driving movement (electric, hydraulic, pneumatic)|
|Sensors|Gather environmental data (e.g., vision, force)|
|Controller|Computes actions based on sensor input and programmed logic|
|Processor|CPU or microcontroller to run algorithms|
|Software|User interface, motion planning, control logic|

---
### **1.5 Robot Configurations and Coordinate Systems**
#### **Coordinate Systems:**

| System      | Description                                |
| ----------- | ------------------------------------------ |
| Cartesian   | Three linear axes (x, y, z)                |
| Cylindrical | One rotation + two translations            |
| Spherical   | Two rotations + one translation            |
| Articulated | Rotary joints (e.g., anthropomorphic arms) |

---
### **1.6 Common Robot Shapes**
- **Manipulators:** Industrial arms with revolute/prismatic joints.
- **Legged Robots:** Quadrupeds, hexapods.
- **Wheeled Robots:** Differential drive, omnidirectional.
- **UAVs (Drones):** Quadcopter, hexacopter.
- **AUVs (Underwater):** Remotely operated vehicles (ROVs).
- **Humanoid Robots:** Human-like body structures.

---
### **1.7 Applications**

✅ **Dangerous environments:**
- Nuclear decontamination
- Deep-sea exploration
- Space missions
✅ **Manual tasks:**
- Assembly, welding
- Material handling
- Automated packaging
✅ **Decontamination and cleaning:**
- Robotic vacuum cleaners
- Window cleaning robots
✅ **Manufacturing:**
- Automotive assembly lines
- PCB soldering