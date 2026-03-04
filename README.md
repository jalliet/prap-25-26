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
- **Card Detector** — identifies playing cards using `vision/models/Card_detection_large_best.pt`
- **Chip Segmentor** — counts chips by colour using `vision/models/Chip_segmentation_large_best.pt`
- **Hand Detector** — player hand tracking (stub, model TBD)

### Arm Controller (ROS 2)
ROS 2 stack for the SO101 robot arm on the `feature/arm_controller` branch:
- LQR trajectory controller with CasADi inverse kinematics
- C++ servo driver for STS3215 servos
- Gazebo simulation with digital twin mode
- See `docs/PLAN_ARM_ROS_BRIDGE.md` for integration plan

### Arm Bridge
`services/arm_ros_bridge.py` — Qt-compatible bridge connecting the main app to the ROS 2 arm controller. Gracefully degrades when ROS 2 is not installed.

## Running

```bash
# Start the dashboard
python main.py

# Run tests
python -m pytest tests/ -v
```

### With ROS 2 Arm (optional)
```bash
# Terminal 1: Start mock arm server (for testing without hardware)
python scripts/mock_arm_server.py

# Terminal 2: Start dashboard
python main.py
```

## Chip Denominations

| Colour | Value |
|--------|-------|
| Red    | 1     |
| Blue   | 5     |
| White  | 20    |
