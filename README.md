# Poker Robot Arm Project (25/26)

The aim of this project is to create a robot arm that can play poker, or act as a dealer in a poker game. In dealer mode, the robot arm will be able to pick up cards, shuffle them, deal them to players, pick up chips, handle the pot and the general actions of the game.
In player mode, the robot arm will be able to pick up cards, play them, and handle chips as needed against other players.

## Getting Started

### Prerequisites

*   **Python 3.12** (Required)
    *   We recommend using [pyenv](https://github.com/pyenv/pyenv) to manage python versions.
*   **Hardware**: Raspberry Pi 5 (8GB RAM) running Raspberry Pi OS (Bookworm).
*   **Camera**: OAK-D Lite Camera (optional, for live feed).
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
Computer vision detectors in `vision/` using YOLOv8 models:
- **Card Detector** — identifies playing cards using `vision/models/Card_detection_large_best.pt`. Live detection can be toggled from the dashboard via the "Toggle Card Detection" button, which overlays bounding boxes and labels on the camera feed.
- **Chip Segmentor** — counts chips by colour using `vision/models/Chip_segmentation_large_best.pt`
- **Hand Detector** — player hand tracking (stub, model TBD)

Model weights are gitignored. Place them in `vision/models/`:
```
vision/models/Card_detection_large_best.pt
vision/models/Chip_segmentation_large_best.pt
```
If weights are missing, detectors run in dummy mode (no inference, no errors).

### Arm Controller (ROS 2)
ROS 2 stack for the SO101 robot arm on the `feature/arm_controller` branch:
- LQR trajectory controller with CasADi inverse kinematics
- C++ servo driver for STS3215 servos
- Gazebo simulation with digital twin mode
- See `docs/PLAN_ARM_ROS_BRIDGE.md` for integration plan

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
- **Toggle Card Detection** — enables live YOLOv8 card detection with bounding box overlays on the camera feed. Detections are logged in the game log.
- **Start/Stop Simulation** — launches or stops the ROS 2 Gazebo simulation (`ros2 launch poker_bringup poker_arm.launch.py mode:=sim`) directly from the GUI.
- **Start Hand / Test Bet** — manual triggers for testing game state transitions.

### With ROS 2 Simulation
Requires a built ROS 2 workspace with the `poker_bringup` package.

```bash
# Build the ROS 2 workspace (once)
colcon build

# Option 1: Launch simulation from the GUI
python main.py
# Then click "Start Simulation" in the dashboard

# Option 2: Launch simulation manually
source install/setup.bash
ros2 launch poker_bringup poker_arm.launch.py mode:=sim
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

## Chip Denominations

| Colour | Value |
|--------|-------|
| Red    | 1     |
| Blue   | 5     |
| White  | 20    |
