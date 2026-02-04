# Poker Arm Control Workspace

This repository hosts the complete **ROS 2 control stack** for the **Poker Arm**, a custom **6-DOF robotic manipulator**.
The system leverages **CasADi** for high-performance symbolic kinematics (FK, IK, Jacobians) and provides a comprehensive dashboard for control, monitoring, and visualisation.

---

## Table of Contents

* [System Requirements](#system-requirements)
* [Installation & Setup](#installation--setup)
* [Quick Start](#quick-start)
* [Operational Modes](#operational-modes)
* [The Dashboard](#the-dashboard)
* [Architecture](#architecture)
* [Troubleshooting](#troubleshooting)

---

## System Requirements

* **OS:** Ubuntu 22.04 (Jammy Jellyfish) or 24.04 (Noble Numbat)
* **ROS 2 Distribution:** Humble Hawksbill or Jazzy Jalisco
* **Python:** 3.10+

### Critical ROS Dependencies
While `rosdep` handles most packages, you must explicitly install the simulation bridges and GUI tools:
```bash
sudo apt install ros-humble-ign-ros2-control ros-humble-ros2-controllers ros-humble-joint-state-publisher-gui

```

---

## Installation & Setup

### 1. Create a Workspace

Create a standard ROS 2 workspace:

```bash
mkdir -p ~/poker_arm_ws/src
cd ~/poker_arm_ws/src

```

### 2. Clone the Repository

Clone the repository into the workspace source folder:

```bash
git clone <YOUR_REPO_URL_HERE> .

```

### 3. Install System Dependencies (rosdep)

Install required ROS 2 dependencies such as `xacro`, `joint_state_publisher`, etc.

```bash
cd ~/poker_arm_ws
sudo apt update
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

```

### 4. Install Python Dependencies

The kinematic solver relies on specific Python libraries.

```bash
cd ~/poker_arm_ws/src/poker_control
pip3 install -r requirements.txt

```

### 5. Build the Workspace

Build the packages using `colcon`:

```bash
cd ~/poker_arm_ws
colcon build --symlink-install

```

### 6. Source the Environment

Source the workspace overlay:

```bash
source install/setup.bash

```

> **Tip:** Add the following line to your `~/.bashrc` for convenience:
> ```bash
> source ~/poker_arm_ws/install/setup.bash
> 
> ```

### 7. Configure USB Latency (Critical for Hardware)

The LQR controller requires a strict **100 Hz (10ms)** loop rate. Standard Linux USB drivers buffer data for **16ms** by default, which physically prevents the loop from running faster than ~60 Hz.

You **must** set the latency timer to **1ms** for the robot's serial port.

**Temporary Fix (Resets on Reboot/Re-plug):**
```bash
# Replace ttyACM0 with your specific port
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyACM0/latency_timer

```

**Permanent Fix (Recommended):**
Create a udev rule to apply this automatically:

```bash
echo 'ACTION=="add", SUBSYSTEM=="usb-serial", ATTR{latency_timer}="1"' | sudo tee /etc/udev/rules.d/99-low-latency.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

```

> **WSL2 Note:** If you are using WSL2, these commands must be run **inside the WSL terminal**, not in Windows.

---

## Quick Start

### Step 1: Generate Kinematic Models

**Crucial:** Before launching any nodes, you must generate the symbolic `.casadi` models.
This script solves the DH parameters and compiles the functions used by the controller.

```bash
ros2 run poker_control generate_kinematics

```

### Step 2: Verify Kinematics

Once models are generated, run the test script to validate the forward and inverse kinematics solvers.

```bash
ros2 run poker_control test_kinematics

```

### Step 3: Launch the System

Bring up the full robot stack (description, controller, and dashboard).
*Note: The launch files are located in the `poker_bringup` package.*

```bash
ros2 launch poker_bringup poker_arm.launch.py mode:=sim

```

### Step 4: Visual Tuning (Optional)

To check the URDF zero-pose or debug geometry without running the full physics engine:

```bash
ros2 launch lerobot_description so101_display.launch.py

```

---

## Operational Modes

The system uses a `mode` launch argument to switch between simulation, hardware, and headless configurations.

### Launch Arguments

* `mode`: Operational context
* `pc_hardware` (default)
* `pi_hardware`
* `pi_hardware_headless`
* `sim`


* `port`: Serial port for hardware driver (default: `/dev/ttyACM0`)

---

### Mode Descriptions

#### 1. PC Hardware (Default)

Runs the full stack on a PC connected to the robot.

* **Nodes:** Controller, Hardware Driver, Dashboard

```bash
ros2 launch poker_bringup poker_arm.launch.py mode:=pc_hardware port:=/dev/ttyACM0

```

---

#### 2. Simulation

Runs the stack in software-only mode.
The hardware driver is replaced by a simulation bridge (Ignition Gazebo).

* **Nodes:** Controller, Sim Bridge, Dashboard

```bash
ros2 launch poker_bringup poker_arm.launch.py mode:=sim

```

---

#### 3. Pi Hardware (Headless)

Optimised for embedded systems (e.g. Raspberry Pi) without a display.

* **Nodes:** Controller, Hardware Driver

```bash
ros2 launch poker_bringup poker_arm.launch.py mode:=pi_hardware_headless

```

---

#### 4. Pi Hardware (With Display)

Same as PC hardware mode, explicitly labelled for embedded use.

* **Nodes:** Controller, Hardware Driver, Dashboard

```bash
ros2 launch poker_bringup poker_arm.launch.py mode:=pi_hardware

```

---

## The Dashboard

The system includes a GUI dashboard for high-level arm control.

### Features

* **Joint Slider Control:** Manual control of joints `q1`–`q6`
* **Cartesian Control:** Target commands in X, Y, Z, Pitch, Roll
* **Solver Feedback:** Displays calculated IK solutions in real-time
* **Status Monitoring:** Real-time joint states and error reporting

> The dashboard is available in `sim`, `pc_hardware`, and `pi_hardware` modes.

---

## Architecture

* **`poker_control/`**
Main package containing source code, custom nodes (Controller, Sim Bridge), and scripts.
* **`poker_bringup/`**
Package containing launch files (`poker_arm.launch.py`).
* **`poker_description/`** (or `lerobot_description`)
URDF/Xacro files defining the robot geometry for RViz/Gazebo.
* **`models/`**
Generated `.casadi` files (FK, IK, Jacobians).
* **`generate_kinematics.py`**
Symbolic kinematics generator.

---

## Troubleshooting

### `ModuleNotFoundError`

Ensure Python dependencies are installed:

```bash
cd ~/poker_arm_ws/src/poker_control
pip3 install -r requirements.txt

```

---

### rosdep Errors

Make sure rosdep has been initialised:

```bash
sudo rosdep init
rosdep update

```

---

### Serial Permission Denied

If hardware mode fails to open the serial port:

```bash
sudo usermod -a -G dialout $USER

```

Log out and log back in for changes to take effect.

---

### Regenerating Models

If you modify DH parameters, delete old models and regenerate:

```bash
rm -rf install/poker_control/share/poker_control/models/*.casadi
colcon build --symlink-install
source install/setup.bash
ros2 run poker_control generate_kinematics

```

---

### Gazebo Crashes on Launch (WSL2)
If Gazebo crashes immediately with an `Ogre::UnimplementedException` or `GL3PlusTextureGpu` error, it is due to WSL2's virtual graphics driver not supporting the required OpenGL features.

Force software rendering before launching:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```