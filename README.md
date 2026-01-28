# Poker Robot Arm Project (25/26)

The aim of this project is to create a robot arm that can play poker, or act as a dealer in a poker game. In dealer mode, the robot arm will be able to pick up cards, shuffle them, deal them to players, pick up chips, handle the pot and the general actions of the game. 
In player mode, the robot arm will be able to pick up cards, play them, and handle chips as needed against other players.

## Getting Started

### Prerequisites

*   Python 3.10+
*   OAK-D Lite Camera (optional, for live feed)

### Installation

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/jalliet/prap-25-26.git
    cd prap-25-26
    ```

2.  **Create a Virtual Environment**:
    ```bash
    python3 -m venv venv
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
