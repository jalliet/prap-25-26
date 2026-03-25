# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

## Project Overview

PRAP 25-26 — a poker-playing robot arm system. PySide6 dashboard with live OAK-D camera vision (YOLOv8 card/chip detection), Texas Hold'em game engine, and ROS 2 integration for a SO-101 6-DOF servo arm (Gazebo simulation or real hardware).

Stack: Python 3.12 / PySide6 / DepthAI / OpenCV / Ultralytics YOLOv8 / ROS 2 Jazzy

## Commands

### Python Application
- Venv: `python3.12 -m venv venv && source venv/bin/activate`
- Install: `pip install -r requirements.txt`
- Run: `python main.py` — starts the PySide6 dashboard (camera, game state, arm controls)

### ROS 2 Workspace
- Build: `source /opt/ros/jazzy/setup.bash && colcon build`
- Source: `source install/setup.bash`
- Launch modes:
  - `ros2 launch poker_bringup poker_arm.launch.py mode:=sim` — Gazebo simulation
  - `ros2 launch poker_bringup poker_arm.launch.py mode:=pc_hardware` — PC + real servos + digital twin
  - `ros2 launch poker_bringup poker_arm.launch.py mode:=pi_hardware` — Raspberry Pi + real servos + digital twin
  - `ros2 launch poker_bringup poker_arm.launch.py mode:=pi_hardware_headless` — Pi headless (no Gazebo, no dashboard)
  - `ros2 launch poker_bringup poker_arm.launch.py dashboard_only:=true` — dashboard only (remote control)

### Mock Testing (no hardware needed)
- Terminal 1: `python scripts/mock_arm_server.py` — fake ROS 2 arm server
- Terminal 2: `python main.py` — dashboard connects to mock

## Architecture

### Entry Point (`main.py`)
Creates QApplication → MainWindow. MainWindow bootstraps GameState, VisionController, ArmRosBridge and wires all signals.

### GUI (`gui/`)
- `main_window.py` — MainWindow (1200x800, QSplitter 30/70)
  - Left: game phase, 5 community card SVG slots, pot display, player list, game log, control buttons
  - Right: Camera FPS spinbox (controls both cameras), vision mode indicator, OAK-D primary feed QLabel (expands), C925e chip feed QLabel (fixed 180px), chip stack result label
  - Buttons: Start Hand, Test Bet, Toggle Card Detection, Start/Stop Simulation
  - Simulation: subprocess `ros2 launch` with SIGINT/SIGTERM cleanup via process groups
- `utils.py` — `convert_cv_qt()` BGR ndarray → QPixmap
- `styles.qss` — dark theme (#2F2F2F bg, emerald pot, blue camera header)

### Services (`services/`)
- `vision_controller.py` — VisionController (Singleton, QObject)
  - Modes: IDLE, CARD_READING (chip segmentation is event-driven, not a mode)
  - Two independent QTimers: `_poll_timer` (OAK-D card detection), `_chip_poll_timer` (C925e chip segmentation)
  - Chip inference is gated by `_chip_inference_active` flag — only runs YOLO after betting actions (CALL/BET/RAISE/ALL_IN) or during SHOWDOWN; timer always streams frames for live preview
  - Deduplication: only emits `cards_detected` when detected set changes (frozenset); only emits `chips_detected` when total changes
  - Signals: `frame_ready(ndarray)`, `cards_detected(list)`, `chip_frame_ready(ndarray)`, `chips_detected(dict)`
  - Auto mode switching: connects to GameState phase/card/action signals
  - Unified `set_fps(fps)` controls both camera timers and hardware FPS
- `birdseye_service.py` — BirdseyeService (OAK-D Lite, overhead/birdseye view, DepthAI)
  - Pipeline: ColorCamera 1080p → preview 1280x720 → XLinkOut
  - Hardware FPS is fixed at pipeline creation (`cam_rgb.setFps()`); `set_fps()` updates config only
  - Graceful fallback: returns None if device unavailable
- `chip_seg_service.py` — ChipSegService (Logitech C925e, chip segmentation view, OpenCV VideoCapture)
  - `ChipSegConfig.device_index=0` — adjust if C925e is not on index 0 (`v4l2-ctl --list-devices` on Linux)
  - Graceful fallback: returns None if device unavailable
- `arm_ros_bridge.py` — ArmRosBridge (Singleton, QObject)
  - Conditional ROS 2 import: `_ROS_AVAILABLE` flag, stubs if rclpy absent
  - Internal `_ArmRosNode` with publishers, subscribers, action clients
  - Publishers: `/target_pose`, `/target_joints`
  - Action clients: `/move_pose` (MovePose), `/move_joints` (MoveJoints)
  - Subscriber: `/motor_feedback` (MotorFeedback)
  - QTimer at 100Hz pumps `rclpy.spin_once()` from Qt thread
  - Qt signals: `connection_changed`, `move_completed`, `move_feedback`, `feedback_received`

### Vision (`vision/`)
- `base_detector.py` — BaseDetector(ABC, Generic[T]): abstract `process()`, YOLO loading with dummy fallback
- `card_detector.py` — CardDetector: YOLOv8 detection → `List[CardDetection]`, dedup by (rank, suit)
- `chip_segmentor.py` — ChipSegmentor: YOLOv8 instance segmentation → ChipStack
- `draw_utils.py` — `draw_card_detections()` bounding box + label overlay
- Models: `vision/models/*.pt` (gitignored) — `Card_detection_large_best.pt`, `Chip_segmentation_large_best.pt`

### Poker Engine (`poker/`)
- `game_state.py` — GameState: custom Signal class, phase FSM, betting logic
  - Phases: PRE_FLOP → FLOP (deal 3) → TURN (deal 1) → RIVER (deal 1) → SHOWDOWN
  - Signals: `on_phase_change`, `on_pot_change`, `on_card_detection_required`, `on_player_action`, `on_turn_change`
- `card.py` — Rank (2-14), Suit (H/D/C/S), Card (hashable), Deck (52 cards)
- `chips.py` — ChipColour (RED=1, BLUE=5, WHITE=20), ChipStack (greedy breakdown via `from_total`)
- `player.py` — Player: PlayerStatus enum, per-hand state, stack management
- `action.py` — ActionType enum (FOLD/CHECK/CALL/BET/RAISE/ALL_IN), Action dataclass

### ROS 2 Workspace (`src/`)
- `poker_bringup/` — Master launch: `poker_arm.launch.py` with mode-conditional node spawning
- `lerobot_description/` — SO-101 URDF/xacro, STL meshes, Gazebo launch (`so101_gazebo.launch.py`)
- `poker_control/` — PokerController node (LQR + CasADi IK, action servers, 100Hz control loop), SimBridge, topic_controller_node
- `poker_dashboard/` — ROS 2 dashboard node (alternative to PySide6 GUI)
- `poker_interfaces/` — Custom msgs (TargetPose, TargetJoints, MotorFeedback, ServoCommand) and actions (MovePose, MoveJoints)
- `scservo_driver/` — C++ STS3215 servo driver node

### Diagrams
- `docs/diagrams/<name>/<name>.mmd` — Mermaid source files, one per subfolder
  - App: system-architecture, vision-pipeline, game-state-fsm, gui-signals, class-relationships
  - ROS 2: ros2-node-graph, launch-modes
- `docs/diagrams/output/<name>/` — rendered PNG/SVG
- `docs/diagrams/render.sh` — render script: `bash render.sh` (all), `bash render.sh -a` (app only), `bash render.sh -r` (ROS 2 only)
- Update diagrams when architecture changes — they are living documentation

### Data Flow
OAK-D Lite → BirdseyeService → VisionController (`_poll_timer`, IDLE/CARD_READING) → CardDetector → `frame_ready`/`cards_detected` → MainWindow.
Logitech C925e → ChipSegService → VisionController (`_chip_poll_timer`, event-driven inference) → ChipSegmentor → `chip_frame_ready`/`chips_detected` → MainWindow.
GameState phase changes → VisionController mode switching (IDLE ↔ CARD_READING). GameState player actions (BET/CALL/RAISE/ALL_IN) → VisionController activates chip inference. GUI arm commands → ArmRosBridge → ROS 2 topics/actions → PokerController → SimBridge/Gazebo or scservo_driver/hardware.

## Conventions

### Git
- Conventional commits: `feat:`, `fix:`, `refactor:`, `docs:`, `chore:`, `perf:`
- Keep commits atomic — one logical change per commit
- Never add "Co-Authored-By" lines to commit messages
- Branch naming: `feature/<name>`

### Python
- Python 3.12+ features allowed
- PySide6 for GUI (not PyQt6)
- Singletons for VisionController and ArmRosBridge (via `__new__`)
- Custom Signal class in `poker/game_state.py` for game events; Qt Signals on QObject subclasses for GUI-bound events
- TypedDict for detection results (CardDetection, BoundingBox, ChipSegmentationResult)
- ABC + Generic[T] for base detector pattern
- Dataclasses for config (`BirdseyeConfig`, `ChipSegConfig` — not frozen, FPS is mutable)
- Type hints throughout

### ROS 2
- Package naming: `poker_*` (poker_bringup, poker_control, poker_interfaces, poker_dashboard)
- Custom messages in poker_interfaces
- Launch arguments for mode switching; conditional node spawning via IfCondition/UnlessCondition + PythonExpression
- rclpy cannot be pip-installed — it comes from the ROS 2 environment, separate from the venv