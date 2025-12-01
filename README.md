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

### 🧠 Numerical Inverse Kinematics: How It Works

Solving Inverse Kinematics (IK) for a hyper-redundant 8-DOF robot like Maverick-9 is non-trivial. Analytical solutions are often undefined due to the infinite number of valid joint configurations for a single end-effector pose. 

To overcome this, we implemented a numerical optimization approach using the **L-BFGS-B** algorithm.

#### The Solver: L-BFGS-B
The **Limited-memory Broyden–Fletcher–Goldfarb–Shanno (Bounded)** algorithm is a quasi-Newton method for optimization.
- **Why this algorithm?** It is highly efficient for high-dimensional problems (8 variables) and natively supports **bound constraints**, ensuring the solver respects the physical joint limits of the robot (e.g., keeping the elbow between $0^{\circ}$ and $90^{\circ}$).
- **How it works:** Instead of solving a closed-form equation, the algorithm treats IK as a minimization problem. It navigates the "terrain" of possible joint configurations to find the "lowest valley"—the state where the error between the robot's hand and the target is zero.

#### The Decision Logic: Multi-Objective Cost Function
The solver minimizes a weighted cost function $C(q)$ to select the single "best" solution among infinite possibilities.

$$C(q) = w_p \| P_{err} \|^2 + w_o \| R_{err} \|^2 + w_{lim} \sum P_{limits} + w_{center} \sum (q - q_{mid})^2$$

| Term | Weight | Description |
| :--- | :---: | :--- |
| **Position Error** ($P_{err}$) | `100.0` | **Primary Objective:** Minimizes the Euclidean distance between the current end-effector position and the target XYZ. The squared term punishes large deviations exponentially. |
| **Orientation Error** ($R_{err}$) | `10.0` | **Secondary Objective:** Uses the Frobenius norm of the rotation matrix difference to align the gripper orientation without suffering from Euler angle gimbal lock. |
| **Joint Limits** ($P_{limits}$) | `1000.0` | **Safety Constraint:** A massive penalty ("soft wall") is applied if the solver attempts to select an angle outside the physical limits defined in `RobotParameters`. |
| **Posture Optimization** ($q_{mid}$) | `0.1` | **Behavioral Bias:** A regularization term that gently nudges the solution toward the center of the joint range. This encourages natural, non-singular poses (e.g., preferring a bent elbow over a locked-straight arm). |

#### Convergence Strategy
1.  **Initial Guess:** We seed the solver with a geometric heuristic (pointing the shoulder yaw toward the target $X,Y$) to start the search in a valid neighborhood.
2.  **Iteration:** The L-BFGS-B algorithm iteratively adjusts the 8 joint angles, calculating the gradient of the cost function to determine the optimal "downhill" direction.
3.  **Validation:** Once converged, the solution is double-checked against a strict tolerance ($<1.0$ cm position error) before being visualized.

### 1. Clone the Repository
```bash
git clone [https://github.com/devArun-13/MAV_9.git](https://github.com/devArun-13/MAV_9.git)
cd MAV_9
