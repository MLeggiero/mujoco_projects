#!/bin/bash
# Quick start script for the room scanner

set -e

# Get directories
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$PROJECT_DIR/ros2_ws"

echo "============================================"
echo "Starting 3D Room Scanner"
echo "============================================"

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS 2 Humble not found. Please install it first."
    exit 1
fi

# Source workspace
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "Error: Workspace not built. Please run build_workspace.sh first."
    exit 1
fi

echo ""
echo "Launching scanner..."
echo ""

# Launch with default settings
ros2 launch room_scanner scanner.launch.py
