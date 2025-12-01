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
