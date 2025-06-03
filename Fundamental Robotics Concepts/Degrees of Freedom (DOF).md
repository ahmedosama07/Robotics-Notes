### **2.1 Introduction to Degrees of Freedom**

A **Degree of Freedom (DOF)** is a measure of the number of independent parameters that define a mechanism’s configuration.

---

### **2.2 DOF for Rigid Bodies**

#### **2.2.1 Rigid Body in 3D Space**
A rigid body in 3D space has **6 DOF**:
$$
\boxed{ \begin{aligned} & \text{Translational:} & x, y, z \\ & \text{Rotational:} & \text{roll (about x), pitch (about y), yaw (about z)} \end{aligned} }
​$$
#### **2.2.2 Rigid Body in 2D Plane**

In the 2D plane:
$$
\boxed{ \text{3 DOF: } x, y \text{ (translations)} \quad \text{and} \quad \theta \text{ (rotation)} }
$$

---
### **2.3 Grübler’s Formula**
#### **2.3.1 Planar Mechanisms**

$$
\boxed{ F = 3(n - 1) - 2j_1 - j_2 }
$$

where:
- $F$: DOF of mechanism
- $n$: number of links (including ground)
- $j_1$​: number of lower pairs (1-DOF joints)
- $j_2$​: number of higher pairs (2-DOF joints)
---
#### **2.3.2 Spatial Mechanisms (Kutzbach Criterion)**
$$
\boxed{ F = 6(n - 1) - 5j_1 - 4j_2 - 3j_3 - 2j_4 - j_5 }
$$
where:
- $j_1$​: 1-DOF pairs
- $j_2$​: 2-DOF pairs
- etc.
---
### **2.4 Detailed Derivations**
#### **2.4.1 Planar Mechanisms Derivation**
Each link in planar motion has 3 DOF.  
For $n$ links:
$$
3n
$$
But one link is grounded:
$$
3(n-1)
$$
Each lower pair (joint with 1 DOF) removes 2 DOF (since 2 relative motions are constrained):
$$
-2j_1
$$
Each higher pair (joint with 2 DOF) removes 1 DOF:
$$
-j_2
$$
Combining:
$$
\boxed{ F = 3(n-1) - 2j_1 - j_2 }
$$
---
#### **2.4.2 Spatial Mechanisms Derivation**
Each link in 3D has 6 DOF:
$$
6n
$$
Ground link:
$$
6(n-1)
$$
Each pair removes relative motion constraints:
- 1-DOF pair: removes 5 DOF
- 2-DOF pair: removes 4 DOF
- etc.
So:
$$
\boxed{ F = 6(n-1) - 5j_1 - 4j_2 - 3j_3 - 2j_4 - j_5}
$$

---
### **2.5 Examples**
#### **Example 2.1: Four-bar Linkage**
Given:
- $n = 4$
- 4 lower pairs ($j_1 = 4$)
- No higher pairs ($j_2=0$)
$$
F = 3(4-1) - 2(4) - 0 = 9 - 8 = 1
$$
$$
\boxed{ \text{Four-bar linkage has 1 DOF} }
$$
---
#### **Example 2.2: Spatial Stewart Platform**
Given:
- $n = 7$ (6 legs + base)
- 6 spherical joints ($j_1=6$)
- 6 universal joints ($j_2=6$)
- No higher pairs
$$
F = 6(7-1) - 5(6) - 4(6) = 36 - 30 - 24 = -18
$$
Since negative, the system is **over-constrained**. The Stewart platform **can** be **fully constrained in practice** due to special geometry, but generally negative $F$ suggests over-constrained structure.
---
#### **Example 2.3: Slider-Crank Mechanism**
Given:
- $n = 4$
- 4 lower pairs ($j_1=4$)
$$
F = 3(4-1) - 2(4) = 9 - 8 = 1
$$$$\boxed{ \text{Slider-crank also has 1 DOF} }
$$
---
### **2.6 Classification Based on DOF**

|DOF Value|Mechanism Type|
|---|---|
|0|Structure (no motion possible)|
|1|Single-DOF mechanism|
|2|Two-DOF mechanism|
|≥3|Multi-DOF (complex) mechanisms|
|Negative|Over-constrained (redundant constraints)|

---
### **2.7 Practical Applications**
- **1-DOF mechanisms:**
	- Four-bar linkages in windshield wipers
- **2-DOF mechanisms:**
	- Universal joints in drive shafts
- **3+ DOF:**
	- 6-DOF industrial manipulators
---
### **Summary Box**
$$
\boxed{ \begin{aligned} & \textbf{Planar:} & F = 3(n-1) - 2j_1 - j_2 \\ & \textbf{Spatial:} & F = 6(n-1) - 5j_1 - 4j_2 - 3j_3 - 2j_4 - j_5 \end{aligned} }
$$​