#!/bin/bash
# Build the ROS 2 workspace

set -e

echo "============================================"
echo "Building ROS 2 Workspace"
echo "============================================"

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$PROJECT_DIR/ros2_ws"

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS 2 Humble not found. Please install it first."
    exit 1
fi

# Navigate to workspace
cd "$WS_DIR"

echo ""
echo "Installing rosdep dependencies..."
rosdep install --from-paths src --ignore-src -r -y || true

echo ""
echo "Building workspace with colcon..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "============================================"
echo "Build complete!"
echo "============================================"
echo ""
echo "To use the workspace, source it:"
echo "  source $WS_DIR/install/setup.bash"
echo ""
echo "Add this to your ~/.bashrc to source automatically:"
echo "  echo 'source $WS_DIR/install/setup.bash' >> ~/.bashrc"
echo ""
