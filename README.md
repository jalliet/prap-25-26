# Discrete LQR Controller & Joint Space Trajectory Generation

This branch implements a **Discrete-Time LQR Controller** for a 6-DOF robotic arm. It utilizes **CasADi** for high-speed kinematics (Forward/Inverse) and generates smooth **Quintic Polynomial** trajectories in joint space to perform pick-and-place motions.

## Key Features
* **Discrete LQR:** Solves the Discrete Algebraic Riccati Equation (DARE) for an integrator system ($\dot{q} = u$).
* **Feedforward Control:** Uses a discrete difference approach for zero-lag tracking of the reference trajectory.
* **CasADi Integration:** Uses pre-compiled `.casadi` functions for efficient FK/IK computations.

## Dependencies

### 1. MATLAB
Tested on MATLAB R2024b.

### 2. CasADi
This project requires the **CasADi** library for optimization and kinematics.
* **Download:** Get the binaries for your OS here: [CasADi Downloads](https://web.casadi.org/get/)
* **Installation:** 1. Unzip the downloaded folder.
    2. Add the folder (and subfolders) to your MATLAB Path:
       ```matlab
       addpath(genpath('/path/to/your/casadi-folder'));
       savepath;
       ```

## Usage Instructions

1.  **Generate Kinematics Functions:**
    Run `generate_arm_kinematics.m` first. This script defines the DH parameters and exports the necessary CasADi functions (`forward_kinematics.casadi`, `inverse_kinematics.casadi`) to the current directory.
    ```matlab
    >> generate_arm_kinematics
    ```

2.  **Run the Simulation:**
    Run `simulation_test.m`. This script:
    * Loads the CasADi functions.
    * Generates a quintic trajectory from a Home position to a Target (calculated via IK).
    * Simulates the closed-loop control.
    * Plots joint tracking errors and animates the task-space trajectory.
    ```matlab
    >> simulation_test
    ```

## File Descriptions

* `simulation_test.m`: The main entry point. Handles controller design, simulation loop, and plotting/animation.
* `generate_arm_kinematics.m`: Symbolic derivation of the robot's kinematics using CasADi. Exports optimized functions.
* `generate_poker_move.m`: Helper function that computes coefficients for a 5th-order polynomial (Quintic) to ensure zero acceleration at start and stop.