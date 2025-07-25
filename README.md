# Project Report: Animation of 3-3R Planar Parallel Manipulator

## 1. Introduction

This project demonstrates the simulation and animation of a **3-3R Planar Parallel Manipulator** using MATLAB. The manipulator consists of three arms, each composed of two rotational joints, connecting a fixed base triangle to a moving triangular platform. This setup finds applications in parallel robotics, precision positioning, and flexible automation systems.

---

## 2. Objective

To create an animated simulation that visualizes the motion of a planar parallel manipulator using inverse kinematics principles, allowing observation of link movement, joint positions, and platform trajectory.

---

## 3. System Overview

* **Manipulator Type**: 3-3R Planar Parallel
* **Joints per Arm**: 2 Revolute Joints
* **Links per Arm**: 2 Equal-length links
* **Platform Motion**: Circular path with angular rotation
* **Inverse Kinematics**: Solved geometrically per frame

---

## 4. Methodology

### 4.1 Parameters Defined

* **Link length (L)** = 20 units
* **Base triangle side (s\_base)** = 50 units
* **Platform triangle side (s\_platform)** = 10 units
* **Animation frames** = 100
* **Platform rotation**: Ranges from 0 to $\frac{\pi}{3}$

### 4.2 Base and Platform Geometry

* The base is a fixed equilateral triangle centered around the origin.
* The platform triangle moves along a circular trajectory (radius = 10 units).
* Each arm connects one base point to the corresponding point on the moving platform.

### 4.3 Inverse Kinematics

* For each arm:

  * Calculate the distance $D$ between base point $B_i$ and platform point $P_i$
  * Check if $D \leq 2L$
  * Compute the midpoint $m$ between $B_i$ and $P_i$
  * Compute perpendicular vector to the line connecting $B_i$ and $P_i$
  * Use geometry to find the elbow joint $J_i$

### 4.4 Animation Details

* Uses MATLAB's `plot`, `drawnow`, and `VideoWriter` functions
* Links and joints are color-coded for clarity:

  * Base to elbow: Red
  * Elbow to platform: Green
  * Joints: Black (Base), Red (Elbow), Blue (Platform)

---

## 5. Code Snapshot (Key Sections)

```matlab
phi = theta_motion(k);
cx = 10 * cos(phi);
cy = 10 * sin(phi);
platform_center = [cx, cy];
...
a = D / 2;
h = sqrt(L^2 - a^2);
m = (B(i,:) + P(i,:)) / 2;
perp = [-d(2), d(1)] / D;
J = m + h * perp;
```

---

## 6. Results

* The animation shows smooth movement of the platform and dynamic adjustment of each arm.
* Arms correctly retract or extend based on platform position.
* Warnings are issued if target point becomes unreachable.

**Output**: A video `animation.avi` is generated showing the complete motion.

---

## 7. Applications

* Simulation of robotic arms for manufacturing
* Demonstrating kinematics in academic research
* Testing control logic for parallel manipulators

---

## 8. Conclusion

This project visualizes the inverse kinematics of a planar parallel manipulator with elegance and clarity. It provides a strong foundation for further extension into real-world robotics, dynamics analysis, or real-time control using sensors and feedback.

---

## 9. References

* Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control*.
* Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2010). *Robotics: Modelling, Planning and Control*.
* MATLAB Documentation: [https://www.mathworks.com/help/matlab/](https://www.mathworks.com/help/matlab/)

---

*Prepared by: \[Your Name]*
*Course/Project: Planar Mechanism Simulation | MATLAB Robotics Project*
