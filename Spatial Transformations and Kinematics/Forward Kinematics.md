### **4.1 Introduction to Forward Kinematics**
Forward kinematics determines the **end-effector position and orientation** from known **joint parameters**.
For a manipulator with joint vector:
$$
\mathbf{q} = \begin{bmatrix} q_1\\q_2\\\vdots\\q_n \end{bmatrix}
$$
the **end-effector pose**:
$$
\mathbf{x} = \begin{bmatrix} x\\y\\z\\\phi\\\theta\\\psi \end{bmatrix}
$$
---
### **4.2 Inverse of Transformation Matrices**
If:
$$
T = \begin{bmatrix} R & \mathbf{p}\\ 0 & 1 \end{bmatrix}
$$
then:
$$
T^{-1} = \begin{bmatrix} R^T & -R^T \mathbf{p}\\ 0 & 1 \end{bmatrix}
$$
---

### **4.3 Frame Assignment Rules**
- **Z-axis** aligned with joint rotation/translation  
- **X-axis** perpendicular to current and previous Z-axes  
- Origin at joint axis intersection
---
### **4.4 Denavit-Hartenberg (DH) Parameters**
For link iii:
- $a_{i-1}$: link length
- $\alpha_{i-1}$: link twist
- $d_i$: link offset
- $\theta_i$: joint angle
---
### **4.5 DH Transformation Matrix**
$$
T_{i-1}^i = \begin{bmatrix} \cos\theta_i & -\sin\theta_i \cos\alpha_{i-1} & \sin\theta_i \sin\alpha_{i-1} & a_{i-1} \cos\theta_i\\ \sin\theta_i & \cos\theta_i \cos\alpha_{i-1} & -\cos\theta_i \sin\alpha_{i-1} & a_{i-1} \sin\theta_i\\ 0 & \sin\alpha_{i-1} & \cos\alpha_{i-1} & d_i\\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
---
### **4.6 Procedure for Forward Kinematics**
1. Identify DH parameters for each link  
2. Compute Ti−1iT_{i-1}^iTi−1i​ for all links  
3. Overall transform:
$$
T_0^n = T_0^1 T_1^2 \cdots T_{n-1}^n
$$
---
### **4.7 Examples**
#### **Example 4.1: 2-DOF Planar Manipulator**
Joint 1: revolute ($\theta_1$)
Joint 2: revolute ($\theta_2$) 
Link lengths: $a_1$, $a_2$ 

---
**DH Table:**

| Link | $a_{i-1}$ | $\alpha_{i-1}$​ | $d_i$ | $\theta_i$​ |
| ---- | --------- | --------------- | ----- | ----------- |
| 1    | 0         | 0               | 0     | $\theta_1$  |
| 2    | $a_1$     | 0               | 0     | $\theta_2$  |

---
**Step 1: $T_0^1$**
$$
T_0^1 = \begin{bmatrix} \cos\theta_1 & -\sin\theta_1 & 0 & 0\\ \sin\theta_1 & \cos\theta_1 & 0 & 0\\ 0 & 0 & 1 & 0\\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
---
**Step 2: $T_1^2$**
$$
T_1^2 = \begin{bmatrix} \cos\theta_2 & -\sin\theta_2 & 0 & a_1\\ \sin\theta_2 & \cos\theta_2 & 0 & 0\\ 0 & 0 & 1 & 0\\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
---
**Step 3: Final Transform**
$$
T_0^2 = T_0^1 T_1^2 = \begin{bmatrix} \cos(\theta_1+\theta_2) & -\sin(\theta_1+\theta_2) & 0 & a_1 \cos\theta_1\\ \sin(\theta_1+\theta_2) & \cos(\theta_1+\theta_2) & 0 & a_1 \sin\theta_1\\ 0 & 0 & 1 & 0\\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
**End-effector position:**
$$
\mathbf{p} = \begin{bmatrix} a_1 \cos\theta_1 + a_2 \cos(\theta_1+\theta_2)\\ a_1 \sin\theta_1 + a_2 \sin(\theta_1+\theta_2)\\ 0 \end{bmatrix}
$$
---
##### **Numerical Example**
Let:
$$
a_1=2, \quad a_2=1, \quad \theta_1=30\degree, \quad \theta_2=45\degree
$$
Compute:
$$
\theta_1+\theta_2=75\degree
$$$$
x = 2\cos30\degree + 1\cos75\degree
$$
$$
y = 2\sin30\degree + 1\sin75\degree
$$
End-effector at:
$$
(1.99, 1.966, 0)
$$
---
#### **Example 4.2: 3-DOF Manipulator (RRP)**
- 2 revolute joints (Joint 1, Joint 2)
- 1 prismatic joint (Joint 3)
- Link lengths $a_1$ and $a_2$
---
DH Table:

| Link | $a_{i-1}$ | $\alpha_{i-1}$ | $d_i$ | $\theta_i$ |
| ---- | --------- | -------------- | ----- | ---------- |
| 1    | 0         | 0              | 0     | $\theta_1$ |
| 2    | $a_1$     | 0              | 0     | $\theta_2$ |
| 3    | $a_2$     | 0              | $d_3$ | 0          |

---
**Step 1: $T_0^1$**
$$
T_0^1 = \begin{bmatrix} \cos\theta_1 & -\sin\theta_1 & 0 & 0\\ \sin\theta_1 & \cos\theta_1 & 0 & 0\\ 0 & 0 & 1 & 0\\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
---
**Step 2: $T_1^2$​**
$$
T_1^2 = \begin{bmatrix} \cos\theta_2 & -\sin\theta_2 & 0 & a_1\\ \sin\theta_2 & \cos\theta_2 & 0 & 0\\ 0 & 0 & 1 & 0\\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
---
**Step 3: $T_2^3$**
Since joint 3 is prismatic ($\theta_3=0$):
$$
T_2^3 = \begin{bmatrix} 1 & 0 & 0 & a_2\\ 0 & 1 & 0 & 0\\ 0 & 0 & 1 & d_3\\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
---
**Step 4: Final Transform**
$$
T_0^3 = T_0^1 T_1^2 T_2^3
$$
Let’s compute:
First, $T_0^1 T_1^2$:
$$
T_0^1 T_1^2 = \begin{bmatrix} \cos(\theta_1+\theta_2) & -\sin(\theta_1+\theta_2) & 0 & a_1 \cos\theta_1\\ \sin(\theta_1+\theta_2) & \cos(\theta_1+\theta_2) & 0 & a_1 \sin\theta_1\\ 0 & 0 & 1 & 0\\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
Now multiply by $T_2^3$:
$$
T_0^3 = \begin{bmatrix} \cos(\theta_1+\theta_2) & -\sin(\theta_1+\theta_2) & 0 & a_1 \cos\theta_1\\ \sin(\theta_1+\theta_2) & \cos(\theta_1+\theta_2) & 0 & a_1 \sin\theta_1\\ 0 & 0 & 1 & 0\\ 0 & 0 & 0 & 1 \end{bmatrix} 
\begin{bmatrix} 1 & 0 & 0 & a_2\\ 0 & 1 & 0 & 0\\ 0 & 0 & 1 & d_3\\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
Multiplication:
$$
T_0^3 = \begin{bmatrix} \cos(\theta_1+\theta_2) & -\sin(\theta_1+\theta_2) & 0 & a_1 \cos\theta_1 + a_2 \cos(\theta_1+\theta_2)\\ \sin(\theta_1+\theta_2) & \cos(\theta_1+\theta_2) & 0 & a_1 \sin\theta_1 + a_2 \sin(\theta_1+\theta_2)\\ 0 & 0 & 1 & d_3\\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
---
**End-Effector Position**
$$
\mathbf{p} = \begin{bmatrix} a_1 \cos\theta_1 + a_2 \cos(\theta_1+\theta_2)\\ a_1 \sin\theta_1 + a_2 \sin(\theta_1+\theta_2)\\ d_3 \end{bmatrix}
$$
---
##### **Numerical Example**
Let:
$$
a_1=2, \quad a_2=1, \quad \theta_1=30\degree, \quad \theta_2=45\degree, \quad d_3=0.5
$$
Compute:
$$
\theta_1 + \theta_2=75\degree
$$$$
x = 2\cos30\degree + 1\cos75\degree
$$
$$
y= 2\sin30\degree + 1\sin75\degree
$$
$$
z= 0.5
$$Final end-effector at:
$$
(1.99, 1.966, 0.5)
$$