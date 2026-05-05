#!/usr/bin/env bash
# Pi-side bringup for the paramiko transport.
#
# Run this on the Raspberry Pi (over SSH) before starting the Mac dashboard.
# Source ROS + venv + workspace install, sanity check pump_test executable,
# then launch the full Pi node graph.
#
# Usage:
#   bash scripts/pi_launch.sh                       # default: pi_hardware_headless
#   bash scripts/pi_launch.sh pi_hardware           # full Pi mode (with dashboard)
#
# Environment overrides:
#   WS_DIR     workspace path (default ~/poker_arm_ws)
#   ROS_SETUP  ROS 2 setup script (default /opt/ros/jazzy/setup.bash)

set -euo pipefail

WS_DIR="${WS_DIR:-$HOME/poker_arm_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
MODE="${1:-pi_hardware_headless}"

cd "$WS_DIR"

# Activate venv (Pi 5 needs rpi-lgpio inside it for pump_test GPIO).
if [ -d ".venv" ]; then
    # shellcheck disable=SC1091
    source .venv/bin/activate
elif [ -d "venv" ]; then
    # shellcheck disable=SC1091
    source venv/bin/activate
else
    echo "ERROR: No virtual environment found in $WS_DIR. Create one with: python3.12 -m venv .venv"
    exit 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"

if [ ! -f "install/setup.bash" ]; then
    echo "ERROR: install/setup.bash missing. Run ./build.sh first."
    exit 1
fi
# shellcheck disable=SC1091
source install/setup.bash

# Sanity check: pump_test must be exposed as a ros2 executable.
if ! ros2 pkg executables poker_control 2>/dev/null | grep -q pump_test; then
    echo "ERROR: pump_test not registered in poker_control. Re-run ./build.sh."
    exit 1
fi

# Sanity check: GPIO backend importable (warns instead of fails so stub mode still launches).
if ! python -c "import RPi.GPIO" 2>/dev/null; then
    echo "WARNING: RPi.GPIO not importable. pump_test will run in stub mode (no hardware)."
    echo "         Install with: pip install rpi-lgpio"
fi

echo "Launching poker_arm in mode=$MODE"
exec ros2 launch poker_bringup poker_arm.launch.py mode:="$MODE"
