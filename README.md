**# MAV_9
This is the 9 DOF System Made for Bosch Fit Fest 2025.
**# Maverick-9: 9-DOF Humanoid Robotic Arm 🦾

![Python](https://img.shields.io/badge/Python-3.9+-blue.svg)
![Status](https://img.shields.io/badge/Status-Prototype-green.svg)
![Event](https://img.shields.io/badge/Event-Bosch%20FIT.Fest%20'25-orange.svg)

**A high-payload, hyper-redundant manipulator featuring a hybrid serial-parallel architecture and proximal actuation strategy.**

---

## 📖 Overview

The **Maverick-9** is a next-generation robotic manipulator designed to address the "Distal Mass" problem in traditional serial arms. Engineered for the **Bosch FIT.Fest '25** challenge, this system separates the heavy motors from the moving joints, shifting weight to the base (Proximal Actuation).

The result is a 9-DOF arm capable of lifting a **5kg payload at full extension (0.7m)** while maintaining high dynamic responsiveness and structural rigidity suitable for mass production.

### 🎥 Digital Twin Demo
![Simulation GUI](Images/GUI.jpg)
*(Real-time Inverse Kinematics solver validating the workspace)*

---

## 🚀 Key Innovations

1.  **Proximal Actuation Strategy:** Heavy motors are mounted near the base or in the forearm, moving power transmission via linear linkages to reduce inertia by ~60%.
2.  **Hybrid Kinematics:** Combines a 5-DOF Serial chain (Shoulder/Elbow) for reach with a 3-DOF Parallel Wrist (3-UPS Tripod) for precision and stiffness.
3.  **9-DOF Redundancy:** 8 active kinematic degrees of freedom + 1 gripper allow for superior obstacle avoidance and singularity management ($8 \text{ DOF} > 6 \text{ Task DOF}$).
4.  **Simulation-First Approach:** A custom Python-based Digital Twin was built to solve the hyper-redundant Inverse Kinematics using numerical optimization before physical prototyping.

---

## ⚙️ System Architecture

| Subsystem | DOF | Actuation | Mechanism |
| :--- | :---: | :--- | :--- |
| **Shoulder** | 3 | 3x CubeMars AK80-64 | Spherical Joint (Yaw-Pitch-Roll) |
| **Elbow** | 2 | 1x NEMA 17 Linear + 1x Servo | Linear Bell-Crank (Lift) + Rotary Stage (Roll) |
| **Wrist** | 3 | 3x Linear Actuators | 3-UPS Parallel Mechanism (Agile Eye) |
| **Gripper** | 1 | 1x Standard Servo | Gear-driven Parallel Jaw |

---

## 💻 Software & Simulation

The kinematics are solved using a custom **Python Digital Twin**. Unlike standard 6-DOF arms, the Maverick-9 requires a numerical approach due to its redundancy.

### Technology Stack
* **Core:** Python 3.9, NumPy
* **Solver:** SciPy (`optimize.minimize` using L-BFGS-B)
* **Visualization:** Matplotlib 3D Animation
* **GUI:** Tkinter

### Inverse Kinematics Logic
We treat the IK problem as an optimization task minimizing a multi-objective cost function:

$$C(q) = w_p \| P_{err} \|^2 + w_o \| R_{err} \|^2 + w_{lim} \sum P_{limits} + w_{center} \sum (q - q_{mid})^2$$

This ensures the robot reaches the target ($P_{err}$) while maintaining orientation ($R_{err}$), respecting mechanical limits ($P_{limits}$), and preferring natural poses ($q_{mid}$).

---

## 🛠️ Installation & Usage

### Prerequisites
* Python 3.8+
* `pip`

### 1. Clone the Repository
```bash
git clone [https://github.com/devArun-13/MAV_9.git](https://github.com/devArun-13/MAV_9.git)
cd MAV_9
