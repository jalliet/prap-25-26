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

### Hand Lifecycle
- **Start Hand** — begins a new poker hand. Emits `on_hand_started` (clears card slots and chip-result label), resets each player's per-hand state, then posts blinds via the existing `process_action` path using `ActionType.POST_SB`/`POST_BB`.

### Betting Controls
Active-player controls split into two rows.

Action row:
- **Fold**, **Check**, **Call**, **Bet**, **Raise**, **All-In** — wired to `GameState.process_action`. Buttons enable/disable based on public `GameState` (phase, current bet, player status, current_bet per seat). Invalid actions fire `on_action_rejected(reason)`, which writes the reason to the game log.

Sizing row:
- `QSpinBox` clamped to `[min_raise, current_player.stack]` for Bet/Raise amounts.
- Four preset buttons that write into the spin box: **1/2 pot**, **pot**, **2x pot**, **all-in**.

### Debug Group
- **Test Bet** — triggers a test betting action for debugging
- **Toggle Card Detection** — enables/disables YOLOv8 card detection with bounding box overlays on the camera feed. Detections are logged when the detected set changes.
- **Start Simulation** — launches `ros2 launch poker_bringup poker_arm.launch.py mode:=sim` as a subprocess
- **Stop Simulation** — sends SIGINT/SIGTERM to the simulation process group
- **Choreographer manual triggers** — Home, Pick Up Deck, Deal to Seat (with seat QSpinBox), Flip Card (with community-index QSpinBox), and Collect Pot. Each calls the matching `ArmChoreographer` entry point so sequences can be tested without `GameState` driving the queue.
- **Sequence status QLabels** — `sequence_status_label` shows the active sequence name and step index (or empty when idle); `sequence_rejection_label` displays the most recent rejection reason from the choreographer.

### Keyboard Shortcuts
- **B** — one-shot birdseye card detection: grabs the current OAK-D frame, runs YOLO card detection, saves annotated PNG to `debug_inference/birdseye/`
- **C** — one-shot chip segmentation: grabs the current C925e frame, runs YOLO chip segmentation, saves annotated PNG to `debug_inference/chip_seg/`

See the Betting Controls above for the in-app action buttons; the B/C shortcuts are debug-only and do not affect game state.

## Camera Feed Behaviour

- Pixmaps scale with `Qt.KeepAspectRatio` and `Qt.SmoothTransformation` (no stretch).
- A 1Hz `QTimer` watches `_last_birdseye_frame_time` and `_last_chip_frame_time`. After 1.5s of frame silence on either camera, MainWindow paints a black QPixmap with centred text ("No signal: OAK-D" / "No signal: C925e") into the affected feed. Live frames resume on the next `frame_ready`/`chip_frame_ready`.

## Styling

Dark theme defined in `styles.qss`