### **3.1 Point Representation in Homogeneous Coordinates**
For transformations in robotics, **homogeneous coordinates** unify translation and rotation into a single matrix representation.
A 3D point:
$$
\mathbf{p} = \begin{bmatrix} x\\y\\z \end{bmatrix}​
$$
Homogeneous form:
$$
\mathbf{p}_h = \begin{bmatrix} x\\y\\z\\1 \end{bmatrix}
$$
---
### **3.2 Vector Representation using Unit Vectors**
A vector **v** in 3D:
$$
\mathbf{v} = \begin{bmatrix} v_x\\v_y\\v_z \end{bmatrix}
$$
Unit vector:
$$
\hat{\mathbf{v}} = \frac{\mathbf{v}}{||\mathbf{v}||}
$$
---
### **3.3 Frame Representation with Normal, Orientation, Approach Vectors**
A typical robot end-effector frame uses:
- **n**: normal vector (x-axis)
- **o**: orientation vector (y-axis)
- **a**: approach vector (z-axis)
$$
R = \begin{bmatrix} n_x & o_x & a_x\\ n_y & o_y & a_y\\ n_z & o_z & a_z \end{bmatrix}
$$
---
### **3.4 Pure Translation Transformation**
For translation by vector $\mathbf{p} = [p_x, p_y, p_z]^T$:
$$T = \begin{bmatrix} I_3 & \mathbf{p} \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} 1 & 0 & 0 & p_x\\ 0 & 1 & 0 & p_y\\ 0 & 0 & 1 & p_z\\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
---
### **3.5 Pure Rotation Transformations**
#### **Rotation about X-axis:**
$$
R_x(\theta)= \begin{bmatrix} 1 & 0 & 0\\ 0 & \cos\theta & -\sin\theta\\ 0 & \sin\theta & \cos\theta \end{bmatrix}
$$
Homogeneous:
$$
T_x(\theta)= \begin{bmatrix} 1 & 0 & 0 & 0\\ 0 & \cos\theta & -\sin\theta & 0\\ 0 & \sin\theta & \cos\theta & 0\\ 0 & 0 & 0 & 1 \end{bmatrix}​​
$$
---
#### **Rotation about Y-axis:**
$$
R_y(\theta)= \begin{bmatrix} \cos\theta & 0 & \sin\theta\\ 0 & 1 & 0\\ -\sin\theta & 0 & \cos\theta \end{bmatrix}
$$
---
#### **Rotation about Z-axis:**
$$R_z(\theta)= \begin{bmatrix} \cos\theta & -\sin\theta & 0\\ \sin\theta & \cos\theta & 0\\ 0 & 0 & 1 \end{bmatrix}
$$
---
### **3.6 Combined Transformations**
$$
T_{\text{combined}} = T_{\text{translation}} \cdot R_z \cdot R_y \cdot R_x
$$
Order of multiplication is crucial:
- **Post-multiplication**: local frame
- **Pre-multiplication**: global frame
---
### **3.7 Inverse of a Transformation Matrix**
For homogeneous transform:
$$
T= \begin{bmatrix} R & \mathbf{p}\\ 0 & 1 \end{bmatrix}
$$
Inverse:
$$
T^{-1}= \begin{bmatrix} R^T & -R^T \mathbf{p}\\ 0 & 1 \end{bmatrix}
$$
---
### **3.8 Properties of Transformation Matrices**
- Orthogonal rotation matrix:
$$
R R^T = I, \quad \det R = 1
$$
- Homogeneous transform determinant:
$$
\det T = 1
$$
- Inverse equals transpose for pure rotations.
---
### **3.9 Examples**
#### **Example 3.1: Translation and Rotation Combination**
Given:
- Translation: $\mathbf{p}=[2,3,4]^T$
- Rotation about Z-axis by $90 \degree$
$$
R_z(90 \degree)= \begin{bmatrix} 0 & -1 & 0\\ 1 & 0 & 0\\ 0 & 0 & 1 \end{bmatrix}
$$
Transformation matrix:
$$
T= \begin{bmatrix} 0 & -1 & 0 & 2\\ 1 & 0 & 0 & 3\\ 0 & 0 & 1 & 4\\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
---
#### **Example 3.2: Inverse Calculation**
Find $T^{-1}$:
$$
R^T= \begin{bmatrix} 0 & 1 & 0\\ -1 & 0 & 0\\ 0 & 0 & 1 \end{bmatrix}, \quad
-R^T\mathbf{p}= \begin{bmatrix} -3\\-2\\-4 \end{bmatrix}
$$
$$
T^{-1}= \begin{bmatrix} 0 & 1 & 0 & -3\\ -1 & 0 & 0 & -2\\ 0 & 0 & 1 & -4\\ 0 & 0 & 0 & 1 \end{bmatrix}
$$
---
#### **Example 3.3: Transformation of a Point**
Point:
$$
\mathbf{p}= \begin{bmatrix} 1\\2\\3\\1 \end{bmatrix}
$$
Transformed:
$$
\mathbf{p}'=T \mathbf{p}
$$
Numerically compute:
$$
\mathbf{p}'= \begin{bmatrix} 0 & -1 & 0 & 2\\ 1 & 0 & 0 & 3\\ 0 & 0 & 1 & 4\\ 0 & 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 1\\2\\3\\1 \end{bmatrix} 
= 
\begin{bmatrix} 0 \cdot 1 + (-1)\cdot 2 + 0\cdot 3 +2\\ 1 \cdot 1 + 0 \cdot 2 + 0 \cdot 3 + 3\\ 0 \cdot 1 + 0 \cdot 2 + 1 \cdot 3 + 4\\ 1 \end{bmatrix} 
=
\begin{bmatrix} 0\\4\\7\\1 \end{bmatrix}
$$​​