# Poker Arm Control Workspace

This repository hosts the complete **ROS 2 control stack** for the **Poker Arm**, a custom **6-DOF robotic manipulator**.
The system leverages **CasADi** for high-performance symbolic kinematics (FK, IK, Jacobians) and provides a comprehensive dashboard for control, monitoring, and visualisation.

---

## Table of Contents

* [System Requirements](#system-requirements)
* [Installation & Setup](#installation--setup)
* [Quick Start](#quick-start)
* [Operational Modes](#operational-modes)
* [Pick-and-flip demo (`poker_demo`)](#pick-and-flip-demo-poker_demo)
* [The Dashboard](#the-dashboard)
* [Architecture](#architecture)
* [Troubleshooting](#troubleshooting)

---

## System Requirements

* **OS:** Ubuntu 22.04 (Jammy Jellyfish) or 24.04 (Noble Numbat)
* **ROS 2 Distribution:** Humble Hawksbill or Jazzy Jalisco
* **Python:** 3.10+
NB: Using this package with WSL2 can cause problems when interfacing with the robot, as it is difficult to give it access to the ports.

### Critical ROS Dependencies
While `rosdep` handles most packages, you must explicitly install the simulation bridges and GUI tools.

**ROS 2 Humble:**
```bash
sudo apt install python3-pip ros-humble-ign-ros2-control ros-humble-ros2-controllers ros-humble-joint-state-publisher-gui
```

**ROS 2 Jazzy:**
```bash
sudo apt install python3-pip ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers ros-jazzy-joint-state-publisher-gui
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
sudo apt install python3-rosdep python3-venv libxcb-cursor0
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Create Virtual Environment and Install Python Dependencies

ROS 2 entry points run with the system Python, so this project uses a build script (`build.sh`) that patches the generated entry points to use the workspace venv after every build. This keeps all Python dependencies (e.g. CasADi) isolated without polluting the system Python.

Create the venv and install dependencies:

```bash
cd ~/poker_arm_ws
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Every new terminal needs your **ROS distro** and this **workspace overlay** (ROS is not sourced automatically when you install it). After the first `./build.sh`, the repo’s helper script can do **venv + overlay** in one step (see **§6**); before that, activate the venv by hand:

```bash
source ~/poker_arm_ws/.venv/bin/activate
```

> **Tip:** Put the ROS line plus `source_ws.sh` in `~/.bashrc` so new shells are ready without manual steps (adjust the distro and path to match your machine):
> ```bash
> source /opt/ros/jazzy/setup.bash
> source ~/poker_arm_ws/source_ws.sh
> ```
> That replaces separate `source .venv/bin/activate` and `source install/setup.bash` once the workspace has been built at least once.

### 5. Build the Workspace

Use the provided `build.sh` script instead of calling `colcon` directly. It runs `colcon build --symlink-install` and then patches the Python entry points to use the venv interpreter — necessary because `colcon` always generates entry points with the system Python shebang.

```bash
cd ~/poker_arm_ws
./build.sh
```

You can pass any `colcon` flags through it:
```bash
./build.sh --packages-select poker_control
```

### 6. Source the Environment

ROS 2 is not loaded globally: you **`source`** it every shell (or via `~/.bashrc`).

**Recommended (after `./build.sh`):** from the workspace root — the folder may be named anything (`poker_arm_ws`, `demo_ws`, …) as long as `.venv`, `install/`, and `source_ws.sh` sit next to each other:

```bash
cd ~/poker_arm_ws
source /opt/ros/jazzy/setup.bash   # or humble — use the distro you installed
source ./source_ws.sh
```

`source_ws.sh` **must be sourced**, not executed (`./source_ws.sh` would run in a subshell and your environment would not change). It activates **`.venv`** and then **`install/setup.bash`**.

**Equivalent by hand:**

```bash
source .venv/bin/activate
source install/setup.bash
```

### 7. Configure USB Latency (Critical for Hardware)

The LQR controller requires a strict **100 Hz (10ms)** loop rate.

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
The hardware driver is replaced by a simulation bridge (Gazebo).

* **Nodes:** Controller, Sim Bridge, Dashboard

```bash
ros2 launch poker_bringup poker_arm.launch.py mode:=sim
```

---

#### 3. Pi Hardware (Headless)

Optimised for embedded systems (e.g. Raspberry Pi) without a display.

* **Nodes:** Controller, Hardware Driver, Poker GPIO bridge

```bash
ros2 launch poker_bringup poker_arm.launch.py mode:=pi_hardware_headless
```

The `poker_gpio` node runs on Raspberry Pi modes and uses BCM **GPIO 22** by
default for the pump (**`pump_pin`**; override if wired differently). The drive
is **active-high by default**: **HIGH** turns the pump **on**, **LOW** turns it **off**
(typical logic-level NMOS gate). GPIO **17** is the button (**`button_pin`**). Use
`pump_active_low:=true` only if your circuit inverts the logical sense (e.g.
active-low coupling).

Pump commands:
```bash
ros2 topic pub --once /pump_control std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /pump_control std_msgs/msg/Int32 "{data: 0}"
```

Button presses publish the circular count on `/button_count`:
```bash
ros2 topic echo /button_count std_msgs/msg/Int32
```

While the controller is executing a trajectory, `/button_count` does not advance (`poker_gpio` watches `/arm_busy` from `poker_controller`). Press the button **after** motion finishes.

#### Pick-and-flip demo (`poker_demo`)

The `poker_demo` package runs a scripted **home**, **pick**, **pump**, and **flip** sequence that calls the `/move_joints` and `/move_pose` actions and publishes to `/pump_control`. Launch it **in a second terminal** after Pi headless hardware is running; it depends on `/button_count`, `/pump_control`, and `/arm_busy` behaving as described above.

**Terminal 1 (stack):**

```bash
ros2 launch poker_bringup poker_arm.launch.py mode:=pi_hardware_headless port:=/dev/ttyACM0
```

**Terminal 2** — after `./build.sh`, with the environment ready (`source /opt/ros/<distro>/setup.bash` and `source ./source_ws.sh`, or the manual venv + `install/setup.bash` pair):

```bash
ros2 run poker_demo pick_demo
```

Optional dwell time after commanding the pump on or off:

```bash
ros2 run poker_demo pick_demo --ros-args \
  -p pump_on_settle_sec:=0.75 \
  -p pump_off_settle_sec:=0.75
```

---

#### 4. Pi Hardware (With Display)

Same as PC hardware mode, explicitly labelled for embedded use.

* **Nodes:** Controller, Hardware Driver, Dashboard, Poker GPIO bridge

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

* **`poker_demo/`**
Scripted dealer demo (`pick_demo`): homes once, drives pick/pump/flip presets, and reacts to `/button_count` when the arm is idle.
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

### `ModuleNotFoundError` for `casadi`, `PySide6`, or other venv packages

This means the entry point was built with the system Python shebang. Always use `./build.sh` instead of `colcon build` — it patches the shebangs automatically after every build.

If you already built with `colcon build`, just run `./build.sh` once to fix it:

```bash
cd ~/poker_arm_ws
./build.sh
source install/setup.bash
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

**Quick fix (current session only):** Grant access to the port immediately without logging out:

```bash
sudo chmod 666 /dev/ttyACM0
```

This resets on reboot or when the device is unplugged. You'll need to re-run it each session.

**Permanent fix:** Add your user to the `dialout` group so the device is always accessible:

```bash
sudo usermod -a -G dialout $USER
```

Log out and log back in for the group change to take effect. After that, `chmod` is no longer needed.

---

### Regenerating Models

If you modify DH parameters, delete old models and regenerate:

```bash
rm -rf install/poker_control/share/poker_control/models/*.casadi
./build.sh
source install/setup.bash
ros2 run poker_control generate_kinematics
```

---

### Gazebo Hangs on Launch — `ros_gz_sim` Loops "Requesting list of world names"

**Symptom:** Gazebo opens a window but the simulation never loads. The terminal repeatedly prints `[ros_gz_sim]: Requesting list of world names.` and the controller spawners time out.

**Cause:** Gazebo Harmonic (shipped with ROS 2 Jazzy) defaults to the Ogre2 renderer, which requires OpenGL 4.3+. On machines without a dedicated GPU — integrated Intel/AMD graphics, VMs, WSL2 — Ogre2 stalls silently during initialisation, blocking the server from ever starting. This is a known Gazebo upstream compatibility issue, not a misconfiguration.

**Fix:** This project's launch file already applies `--render-engine ogre` (Ogre1, requires only OpenGL 2.1) and sets `GZ_IP=127.0.0.1` (pins gz-transport to loopback). No action needed — the launch file handles it automatically.

If you have forked or modified the launch files and see this issue, add these two lines to `so101_gazebo.launch.py`:

```python
SetEnvironmentVariable(name="GZ_IP", value="127.0.0.1")
```

```python
("gz_args", [" -v 4 -r empty.sdf --render-engine ogre"])
```

---

### Gazebo Crashes on Launch (WSL2)
If Gazebo crashes immediately with an `Ogre::UnimplementedException` or `GL3PlusTextureGpu` error, it is due to WSL2's virtual graphics driver not supporting the required OpenGL features.

Force software rendering before launching:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```