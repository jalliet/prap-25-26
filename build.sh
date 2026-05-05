#!/bin/bash
# Wrapper around colcon build that patches entry point shebangs to use the
# workspace venv. Required because colcon regenerates entry points with the
# system Python shebang, but our dependencies (e.g. casadi) live in the venv.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_PYTHON="${SCRIPT_DIR}/.venv/bin/python3"

colcon build --symlink-install "$@"

patch_package() {
    local pkg="$1"
    shift
    for ep in "$@"; do
        local target="${SCRIPT_DIR}/install/${pkg}/lib/${pkg}/${ep}"
        if [ -f "$target" ]; then
            sed -i "1s|.*|#!${VENV_PYTHON}|" "$target"
        fi
    done
}

patch_package poker_control controller generate_kinematics sim_bridge test_kinematics
patch_package poker_dashboard dashboard
patch_package poker_gpio poker_gpio

echo "Shebangs patched to use venv Python."
