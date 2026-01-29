# Poker Robot Arm Project (25/26)

The aim of this project is to create a robot arm that can play poker, or act as a dealer in a poker game. In dealer mode, the robot arm will be able to pick up cards, shuffle them, deal them to players, pick up chips, handle the pot and the general actions of the game. 
In player mode, the robot arm will be able to pick up cards, play them, and handle chips as needed against other players.

## Getting Started

### Prerequisites

*   **Python 3.11** (Required)
    *   This version is standard on Raspberry Pi OS (Bookworm) and ensures compatibility with `PySide6` and `depthai`.
    *   We recommend using [pyenv](https://github.com/pyenv/pyenv) to manage python versions.
*   **Hardware**: Raspberry Pi 5 (8GB RAM) running Raspberry Pi OS (Bookworm).
*   **Camera**: OAK-D Lite Camera (optional, for live feed).

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
    
    Using standard python (ensure it is 3.11):
    ```bash
    python3.11 -m venv venv
    ```
    
    OR using pyenv:
    ```bash
    pyenv install 3.11
    pyenv local 3.11
    python -m venv venv
    ```

3.  **Activate the Virtual Environment**:
    ```bash
    # On macOS/Linux
    source venv/bin/activate
    # On Windows
    # .\venv\Scripts\activate
    ```

4.  **Install Dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

## Components

### [Dashboard GUI](gui/README.md)
The graphical interface for monitoring the game state and camera feed. See the [GUI Documentation](gui/README.md) for running instructions and feature details.
