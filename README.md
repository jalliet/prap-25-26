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
Core game logic in `poker/` — card/deck management, chip stacks, player state, betting actions, and game phase transitions (Pre-Flop → Showdown). Hand evaluation uses the `phevaluator` library (`poker/evaluator.py`); side pots are computed via a capped-contribution algorithm at showdown.

### Vision System
Dual-camera computer vision pipeline using YOLOv8 models:
- **Card Detector** (`OAK-D Lite`) — identifies playing cards using `vision/models/Card_detection_large_best.pt`. Detection can be toggled via "Toggle Card Detection" in the dashboard, which overlays bounding boxes and labels on the primary feed.
- **Chip Segmentor** (`Logitech C925e`) — counts chips by colour using `vision/models/Chip_segmentation_large_best.pt`. Runs on the dedicated secondary camera with event-driven inference — YOLO only triggers after betting actions (call/bet/raise/all-in) or during showdown; the live preview always streams. The chip stack total is shown in the right panel.

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
- **Debug Inference (keyboard)** — press **B** to run one-shot card detection on the OAK-D feed and save the annotated frame to `debug_inference/birdseye/`. Press **C** for chip segmentation on the C925e feed, saved to `debug_inference/chip_seg/`. Timestamped PNGs are saved and the path is logged.

#### Betting Controls
The left panel exposes operator-facing controls for the active player:
- **Action row** — Fold, Check, Call, Bet, Raise, All-In. Buttons enable or disable based on the current `GameState` (phase, current bet, player status). Invalid actions are routed through the `on_action_rejected` signal and surfaced in the game log.
- **Sizing row** — `QSpinBox` (clamped to `[min_raise, current_player.stack]`) plus four preset buttons that write into the spin box: 1/2 pot, pot, 2x pot, all-in.

Existing test buttons (`Test Bet`, `Toggle Card Detection`, `Start/Stop Simulation`) live in a Debug group below the real controls.

#### Hardware Integration (pi_hardware modes)
Under `mode:=pi_hardware` and `mode:=pi_hardware_headless`, the launch file spawns `pump_test` (a ROS 2 GPIO node in `poker_control`) for the table-side button and suction pump:
- **Table button (GPIO 27)** — RISING-edge increments a seat counter and publishes `Int32` on `/button_count`. The dashboard's `services/table_io_bridge.py` subscribes and calls `GameState.next_turn()` to advance the turn.
- **Suction pump (GPIO 17)** — `pump_test` subscribes to `/pump_control` (Int32) and toggles GPIO 17 HIGH or LOW based on `msg.data`. `ArmChoreographer` builds pump-on and pump-off steps into its `pick_up_deck` and `deal_card_to_seat` sequences, emitting `pump_requested(bool)` at each pump step. `MainWindow` routes that signal to `TableIoBridge.set_pump`, which publishes `Int32(1)` or `Int32(0)` on `/pump_control` and signals `pump_state_set(bool)` once the `pump_duration_s` settle delay (default 0.05) elapses. The choreographer waits on `pump_state_set` before moving past the pump step. Physical chip movement stays manual.

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

### Choreographer Status in the GUI

The dashboard surfaces choreographer state through two read-only labels at the top of the right (camera) panel:
- **`sequence_status_label`** shows the active sequence name and the current step index (for example `Sequence: pick_up_deck (step 2)`), or empty when no sequence is running.
- **`sequence_rejection_label`** displays the most recent rejection reason from the choreographer (for example `Last rejection: pick_up_deck`) so operators can see why a sequence was refused.

Manual triggers live in the Debug QGroupBox on the left panel for integration testing without `GameState` driving the queue:
- **Home** button calls `choreographer.home()`.
- **Pick Up Deck** button calls `choreographer.pick_up_deck()`.
- **Deal to Seat** button with a seat-index QSpinBox calls `choreographer.deal_card_to_seat(seat)`.
- **Flip Card** button with a community-index QSpinBox (0..4) calls `choreographer.flip_card(num_players + i)`.
- **Collect Pot** button calls `choreographer.collect_pot()`.

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
