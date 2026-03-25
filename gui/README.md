# Poker Robot Dashboard GUI

PySide6-based dashboard for the Poker Robot Arm Project. See the [Root README](../README.md) for environment setup.

## Running

```bash
source venv/bin/activate
python main.py
```

## Layout

1200x800 window with a 30/70 QSplitter:

- **Left Panel**: Game phase, 5 community card SVG slots, pot display, player list, game log, control buttons
- **Right Panel**:
  - Camera FPS spinbox (1-60, controls both cameras) + vision mode indicator
  - Primary OAK-D Lite feed (QLabel, expands to fill available height) — birdseye card detection
  - Chip Camera (C925e) header + compact 180px feed (QLabel) — event-driven chip segmentation (inference triggers on betting actions and showdown; live preview always streams)
  - Chip stack result label (updates when total chip value changes)

## Controls

- **Start Hand** — begins a new poker hand, resets game state
- **Test Bet** — triggers a test betting action for debugging
- **Toggle Card Detection** — enables/disables YOLOv8 card detection with bounding box overlays on the camera feed. Detections are logged when the detected set changes.
- **Start Simulation** — launches `ros2 launch poker_bringup poker_arm.launch.py mode:=sim` as a subprocess
- **Stop Simulation** — sends SIGINT/SIGTERM to the simulation process group

## Styling

Dark theme defined in `styles.qss`