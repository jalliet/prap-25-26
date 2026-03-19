# Poker Robot Arm Project (25/26)

The aim of this project is to create a robot arm that can play poker, or act as a dealer in a poker game. In dealer mode, the robot arm will be able to pick up cards, shuffle them, deal them to players, pick up chips, handle the pot and the general actions of the game.
In player mode, the robot arm will be able to pick up cards, play them, and handle chips as needed against other players.

## Getting Started

### Prerequisites

*   **Python 3.12** (Required)
    *   We recommend using [pyenv](https://github.com/pyenv/pyenv) to manage python versions.
*   **Hardware**: Raspberry Pi 5 (8GB RAM) running Raspberry Pi OS (Bookworm).
*   **Cameras**: OAK-D Lite (birdseye card detection) + Logitech C925e (chip segmentation)
*   **Robot Arm**: SO101 6-DOF servo arm (optional, for arm control).

### Installation

1.  **Clone the repository**:

    Using HTTPS:
    ```bash
    git clone https://github.com/jalliet/prap-25-26.git
    cd prap-25-26
    ```

    Using SSH:
    ```bash
    git clone git@github.com:jalliet/prap-25-26.git
    cd prap-25-26
    ```

2.  **Create a Virtual Environment**:

    Using standard python (ensure it is 3.12):
    ```bash
    python3.12 -m venv venv
    ```

    OR using pyenv:
    ```bash
    pyenv install 3.12
    pyenv local 3.12
    python -m venv venv
    ```

3.  **Activate the Virtual Environment**:
    ```bash
    source venv/bin/activate
    ```

4.  **Install Dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

## Components

### Dashboard GUI
The PySide6 graphical interface for monitoring the game state and camera feed. See the [GUI Documentation](gui/README.md) for details.

### Poker Engine
Core game logic in `poker/` — card/deck management, chip stacks, player state, betting actions, and game phase transitions (Pre-Flop → Showdown).

### Vision System
Dual-camera computer vision pipeline using YOLOv8 models:
- **Card Detector** (`OAK-D Lite`) — identifies playing cards using `vision/models/Card_detection_large_best.pt`. Detection can be toggled via "Toggle Card Detection" in the dashboard, which overlays bounding boxes and labels on the primary feed.
- **Chip Segmentor** (`Logitech C925e`) — counts chips by colour using `vision/models/Chip_segmentation_large_best.pt`. Runs as an always-on background pipeline on the dedicated secondary camera; the chip stack total is shown in the right panel.

Model weights are gitignored. Place them in `vision/models/`:
```
vision/models/Card_detection_large_best.pt
vision/models/Chip_segmentation_large_best.pt
```
If weights are missing, detectors run in dummy mode (no inference, no errors).

### Arm Controller (ROS 2)
ROS 2 workspace in `src/` for the SO-101 robot arm:
- **poker_bringup** — master launch file with 4 modes (sim, pc_hardware, pi_hardware, pi_hardware_headless)
- **poker_control** — LQR trajectory controller with CasADi inverse kinematics, action servers
- **poker_interfaces** — custom ROS 2 messages (TargetPose, TargetJoints, MotorFeedback) and actions (MovePose, MoveJoints)
- **lerobot_description** — SO-101 URDF/xacro, STL meshes, Gazebo launch
- **scservo_driver** — C++ driver for STS3215 servos
- **poker_dashboard** — ROS 2 dashboard node (alternative to PySide6 GUI)

### Arm Bridge
`services/arm_ros_bridge.py` — Qt-compatible bridge connecting the main app to the ROS 2 arm controller. Gracefully degrades when ROS 2 is not installed.

## Running

### Dashboard Only
```bash
# Start the dashboard (camera feed, card detection, game state)
python main.py

# Or use the helper script (checks Python version)
bash scripts/start_game.sh
```

### Dashboard Features
- **Toggle Card Detection** — enables live YOLOv8 card detection with bounding box overlays on the primary OAK-D feed. Detections are logged in the game log. Two camera feeds are visible in the right panel: the primary OAK-D birdseye feed (top) and the compact C925e chip feed (bottom).
- **Start/Stop Simulation** — launches or stops the ROS 2 Gazebo simulation (`ros2 launch poker_bringup poker_arm.launch.py mode:=sim`) directly from the GUI.
- **Start Hand / Test Bet** — manual triggers for testing game state transitions.

### With ROS 2 Simulation
Requires ROS 2 Jazzy and a built workspace.

```bash
# Source ROS 2 and build the workspace (once)
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

# Option 1: Launch simulation from the GUI
python main.py
# Then click "Start Simulation" in the dashboard

# Option 2: Launch simulation manually
ros2 launch poker_bringup poker_arm.launch.py mode:=sim
```

### ROS 2 Launch Modes
```bash
ros2 launch poker_bringup poker_arm.launch.py mode:=sim                # Gazebo simulation only
ros2 launch poker_bringup poker_arm.launch.py mode:=pc_hardware        # PC + real servos + digital twin
ros2 launch poker_bringup poker_arm.launch.py mode:=pi_hardware        # Raspberry Pi + real servos + digital twin
ros2 launch poker_bringup poker_arm.launch.py mode:=pi_hardware_headless  # Pi headless (no Gazebo, no dashboard)
ros2 launch poker_bringup poker_arm.launch.py dashboard_only:=true     # Dashboard only (remote control)
```

### With Mock Arm Server (no ROS 2 needed)
```bash
# Terminal 1: Start mock arm server
python scripts/mock_arm_server.py

# Terminal 2: Start dashboard
python main.py
```

### Running Tests
```bash
python -m pytest tests/ -v
```

## Architecture Diagrams

Mermaid diagrams in `docs/diagrams/`, one per subfolder:

| Diagram | Description |
|---------|-------------|
| system-architecture | High-level component map (GUI, services, vision, poker, ROS 2) |
| vision-pipeline | Camera → detectors → dedup → signals → display |
| game-state-fsm | Poker phase transitions (Pre-Flop → Showdown) |
| ros2-node-graph | ROS 2 nodes, topics, and action servers |
| gui-signals | Qt/custom signal connections between components |
| launch-modes | Which nodes spawn per ROS 2 launch mode |
| class-relationships | Class diagram with inheritance and composition |

Render all diagrams:
```bash
bash docs/diagrams/render.sh       # all diagrams
bash docs/diagrams/render.sh -a    # app diagrams only
bash docs/diagrams/render.sh -r    # ROS 2 diagrams only
```

Requires `npx` (Node.js). Output goes to `docs/diagrams/output/`.

## Chip Denominations

| Colour | Value |
|--------|-------|
| Red    | 1     |
| Blue   | 5     |
| White  | 20    |
