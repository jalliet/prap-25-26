# Poker Robot Dashboard GUI

This directory contains the PySide6-based Graphical User Interface for the Poker Robot Arm Project.

## Prerequisites

*   Python 3.10+
*   OAK-D Lite Camera (optional, for live feed)

## Setup

1.  **Create a Virtual Environment** (if you haven't already):
    ```bash
    # From the project root
    python3 -m venv venv
    ```

2.  **Activate the Virtual Environment**:
    ```bash
    source venv/bin/activate
    ```

3.  **Install Dependencies**:
    ```bash
    pip install -r ../requirements.txt
    ```

## Running the Application

To launch the dashboard, run the start script from the project root:

```bash
./scripts/start_game.sh
```

Or run the entry point manually:

```bash
source venv/bin/activate
python3 main.py
```

## Features

*   **Game State Panel**: Displays community cards, pot size, player list, and game logs.
*   **Camera Feed**: Live 1080p preview from the OAK-D Lite camera.
*   **FPS Control**: Adjust the camera refresh rate (1-60 FPS) via the UI spinner.
