#!/usr/bin/env bash
# Activate the workspace venv and overlay this package's install space.
#
# Must be *sourced* so the environment applies to your current shell:
#   source ./source_ws.sh
#
# Prerequisite (each new terminal, before this script): source your ROS distro, e.g.
#   source /opt/ros/humble/setup.bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "error: source this file instead of executing it:"
    echo "  source ./source_ws.sh"
    exit 1
fi

_WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_VENV_ACT="${_WS_ROOT}/.venv/bin/activate"
_INSTALL_SETUP="${_WS_ROOT}/install/setup.bash"

if [[ ! -f "${_VENV_ACT}" ]]; then
    echo "error: missing venv: ${_VENV_ACT}" >&2
    echo "  Create it with: python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt" >&2
    unset _WS_ROOT _VENV_ACT _INSTALL_SETUP
    return 1 2>/dev/null || exit 1
fi

if [[ ! -f "${_INSTALL_SETUP}" ]]; then
    echo "error: missing ${_INSTALL_SETUP} — build the workspace first:" >&2
    echo "  ./build.sh" >&2
    unset _WS_ROOT _VENV_ACT _INSTALL_SETUP
    return 1 2>/dev/null || exit 1
fi

# shellcheck source=/dev/null
source "${_VENV_ACT}"
# shellcheck source=/dev/null
source "${_INSTALL_SETUP}"

unset _WS_ROOT _VENV_ACT _INSTALL_SETUP
