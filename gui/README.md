# Poker Robot Dashboard GUI

This directory contains the PySide6-based Graphical User Interface for the Poker Robot Arm Project.

## Setup

Please refer to the [Root README](../README.md) for environment setup and installation instructions.

## Running the Application

Ensure your virtual environment is activated:

```bash
source ../venv/bin/activate
```

To launch the dashboard, run the start script from the project root:

```bash
./scripts/start_game.sh
```

Or run the entry point manually:

```bash
python3 main.py
```

## Features

*   **Game State Panel**: Displays community cards, pot size, player list, and game logs.
*   **Camera Feed**: Live 1080p preview from the OAK-D Lite camera.
*   **FPS Control**: Adjust the camera refresh rate (1-60 FPS) via the UI spinner.
