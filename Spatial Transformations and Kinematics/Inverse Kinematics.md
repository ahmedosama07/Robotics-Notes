### **5.1 What is Inverse Kinematics?**
- **Forward Kinematics (FK)**: Given joint variables $q$, find end-effector pose $x$.
- **Inverse Kinematics (IK)**: Given desired pose $x$, find joint variables $q$.
- IK is generally **non-linear** and may have:
    - **Multiple solutions**
    - **No solution** (if target unreachable)
---
### **5.2 Geometric Method**
For **3-DOF manipulators**, we often start with a **geometric approach** using **trigonometry**.
#### **Example 5.1: 3-DOF Planar Manipulator**
- Links: $l_1$, $l_2$, $l_3$
- Joint angles: $\theta_1$, $\theta_2$, $\theta_3$
- End-effector $(x, y)$ and orientation $\phi$.
**Position Equations:**
$$
\begin{cases}
x = l_1 \cos\theta_1 + l_2 \cos(\theta_1 + \theta_2) + l_3 \cos(\theta_1 + \theta_2 + \theta_3) \\
y = l_1 \sin\theta_1 + l_2 \sin(\theta_1 + \theta_2) + l_3 \sin(\theta_1 + \theta_2 + \theta_3)
\end{cases}
$$
**Orientation Equation:**
$$
\phi = \theta_1 + \theta_2 + \theta_3
$$
---
#### **Step-by-Step IK Solution**
#### **Step 1: Compute $\theta_1$**:
Position of wrist center $(x_c, y_c)$:
$$
x_c = x - l_3 \cos\phi
$$
$$
y_c = y - l_3 \sin\phi
$$
#### **Step 2: Compute $\theta_2$**:
Using the law of cosines:
$$
r^2 = x_c^2 + y_c^2
$$
$$
\cos\theta_2 = \frac{r^2 - l_1^2 - l_2^2}{2 l_1 l_2}
$$
$$
\theta_2 = \tan^{-1}(\pm \sqrt{1 - \cos^2\theta_2}, \cos\theta_2)
$$
#### **Step 3: Compute θ1\theta_1θ1​**:
$$
\beta = \tan^{-1}(y_c, x_c)
$$
$$
\gamma = \tan^{-1}(l_2 \sin\theta_2, l_1 + l_2 \cos\theta_2)
$$
$$
\theta_1 = \beta - \gamma
$$
#### **Step 4: Compute $\theta_3$**:
$$
\theta_3 = \phi - (\theta_1 + \theta_2)
$$
---
### **5.3 Algebraic Method**
For 3-DOF **spatial manipulators** (not planar), you use the **Denavit-Hartenberg (DH)** convention.
Example DH transformations:
$$
T_i^{i-1} = \begin{bmatrix} \cos\theta_i & -\sin\theta_i \cos\alpha_{i-1} & \sin\theta_i \sin\alpha_{i-1} & a_{i-1} \cos\theta_i \\ \sin\theta_i & \cos\theta_i \cos\alpha_{i-1} & -\cos\theta_i \sin\alpha_{i-1} & a_{i-1} \sin\theta_i \\ 0 & \sin\alpha_{i-1} & \cos\alpha_{i-1} & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
**Given desired pose matrix** $T_{06}$, extract position and orientation, and solve for $\theta_1, \theta_2, \theta_3$.  
This typically involves:
- Equating rotation matrices
- Equating position vector equations
- Using inverse trigonometric functions and algebraic manipulation
---
### **5.4 Examples**
#### **Example 5.1: 3-DOF Planar Arm**
**Given:**
- $l_1 = 1 \, \text{m}$, $l_2 = 1 \, \text{m}$, $l_3 = 0.5 \, \text{m}$
- Desired end-effector position: $(x, y) = (1.5, 0.5)$
- Desired orientation: $\phi = 30\degree = \pi/6$
---
**Step 1: Compute wrist center**
$$
x_c = 1.5 - 0.5 \cos(\pi/6)
$$
$$
y_c = 0.5 - 0.5 \sin(\pi/6)
$$
**Step 2: Compute $\theta_2$**:
$$
r^2 = {x_c}^2 + {y_c}^2
$$
$$
\cos\theta_2 = \frac{r - {l_1}^2 - {l_2}^2}{2 \times {l_1} \times {l_2}}
$$
$$
\theta_2 = \pm \cos^{-1}{(-0.4)}
$$
**Step 3: Compute $\theta_1$**:
$$
\gamma = l_1 + l2 \cdot \cos{\theta_2}
$$
$$
\beta = l_2 \cdot \sin{theta_2}
$$
$$
\cos{\theta_1} = \frac{\gamma x_c + \beta y_c}{\gamma^2 + \beta^2}
$$
$$
\sin{\theta_1} = \frac{\gamma y_c - \beta x_c}{\gamma^2 + \beta^2}
$$

- **Solution 1 (Elbow-up, $\theta_2 > 0$):**
$$
\theta_1 = -0.76
$$
- **Solution 2 (Elbow-down, $\theta_2 < 0$):**
$$
\theta_1 = 1.22
$$
**Step 4: Compute $\theta_3$**:
$$
\theta_3 = \phi - (\theta_1 + \theta_2)
$$- **Solution 1 (Elbow-up, $\theta_2 > 0$):**
$$
\theta_3 = -0.69
$$
- **Solution 2 (Elbow-down, $\theta_2 < 0$):**
$$
\theta_3 = 1.27
$$
---
#### **Example 5.2: 6-DOF Manipulator**
![[PUMA 560.png]]
Given the **PUMA 560 DH Parameters**:

| Link | $a_{i-1}$ | $\alpha_{i-1}$ | $d_i$ | $\theta_i$ |
| ---- | --------- | -------------- | ----- | ---------- |
| 1    | 0         | −π/2-\pi/2−π/2 | $d_1$ | $\theta_1$ |
| 2    | $a_1$     | 0              | 0     | $\theta_2$ |
| 3    | $a_2$     | $-\pi/2$       | $d_3$ | $\theta_3$ |
| 4    | 0         | $\pi/2$        | $d_4$ | $\theta_4$ |
| 5    | 0         | $-\pi/2$       | 0     | $\theta_5$ |
| 6    | 0         | 0              | $d_6$ | $\theta_6$ |

---
###### **Decoupling via Wrist Center**
Desired end-effector transformation
$$
T_0^6 = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix}
$$**Wrist center** position:
$$
p_w = p - d_6 R \hat{z}_6
$$
where:
- $p$: desired end-effector position
- $R$: desired orientation
- $\hat{z}_6 = [0, 0, 1]^T$ in end-effector frame
---
###### **Solving for $\theta_1, \theta_2, \theta_3$**
$\theta_1$:
From the wrist center projection in base frame:
$$
\theta_1 = \tan^{-1}\left( \frac{y_w}{x_w} \right)
$$
$\theta_3$:
Form a triangle using links $a_2$, $d_4$ and the projection from joint 2 to the wrist center.
Let:
$$
r = \sqrt{x_w^2 + y_w^2}
$$
$$
s = z_w - d_1
$$
Using cosine law:
$$
D = \frac{r^2 + s^2 - a_2^2 - d_4^2}{2 a_2 d_4}
$$
$$
\theta_3 = \tan^{-1}(\pm \sqrt{1 - D^2}, D)
$$
$\theta_2$:
$$
\beta = \tan^{-1}\left( \frac{s}{r} \right)
$$
$$
\gamma = \tan^{-1}\left( \frac{d_4 \sin\theta_3}{a_2 + d_4 \cos\theta_3} \right)
$$
$$
\theta_2 = \beta - \gamma
$$
---
###### **Solving for Wrist Joint Angles $\theta_4$, $\theta_5$, $\theta_6$​**
Calculate:
$$
R_0^3 = R_z(\theta_1) R_x(-\pi/2) R_z(\theta_2) R_z(\theta_3)
$$ Or multiply corresponding DH matrices.

**Wrist rotation matrix**
$$
R_3^6 = (R_0^3)^T R
$$

$\theta_4$, $\theta_5$, $\theta_6$
From:
$$
R_3^6 = \begin{bmatrix} c_5 c_6 & -c_5 s_6 & s_5 \\ c_4 s_6 + s_4 s_5 c_6 & c_4 c_6 - s_4 s_5 s_6 & -s_4 c_5 \\ s_4 s_6 - c_4 s_5 c_6 & s_4 c_6 + c_4 s_5 s_6 & c_4 c_5 \end{bmatrix}
$$
Extract:
$$
\theta_5 = \tan^{-1}( \sqrt{r_{13}^2 + r_{23}^2}, r_{33} )
$$
$$
\theta_4 = \tan^{-1}( r_{23}, r_{13} )
$$
$$
\theta_6 = \tan^{-1}( -r_{32}, r_{31} )
$$---
##### **Numerical Example**
Let:
- $a_2=0.4\, \text{m}$, $d_4=0.3\, \text{m}$, $d_1=0.5\, \text{m}$
- Desired end-effector position: $p=[0.3, 0.2, 0.8]^T$
- Desired orientation: $R = I$

##### **Compute wrist center**
Assume $d_6=0.1\, \text{m}$
$$
p_w = p - 0.1 [0, 0, 1]^T = [0.3, 0.2, 0.7]
$$
$\theta_1$:
$$
\theta_1 = \tan^{-1}\left( \frac{0.2}{0.3} \right)
$$$\theta_3$:
$$
r = \sqrt{0.3^2 + 0.2^2}
$$
$$
s = 0.7 - 0.5 = 0.2
$$$$
D = \frac{r^2 + s^2 - a_2^2 - d_4^2}{2 a_2 d_4} = -0.33
$$
$$
\theta_3 = \tan^{-1}(\pm \sqrt{1 - D^2}, D) \approx -19\degree
$$
$\theta_2$:
$$
\beta = \tan^{-1}\left( \frac{s}{r} \right)
$$
$$
\gamma = \tan^{-1}\left( \frac{d_4 \sin\theta_3}{a_2 + d_4 \cos\theta_3} \right)
$$
$$
\theta_2 = \beta - \gamma = 37\degree
$$