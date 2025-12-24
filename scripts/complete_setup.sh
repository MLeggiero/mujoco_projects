#!/bin/bash
# Complete setup script - runs all installation steps in order

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "======================================================="
echo "3D Room Scanner - Complete Setup"
echo "======================================================="
echo ""
echo "This script will:"
echo "  1. Install ROS 2 Humble"
echo "  2. Install all dependencies (RealSense, RTABMap, etc.)"
echo "  3. Setup USB permissions for RealSense"
echo "  4. Build the ROS 2 workspace"
echo ""
read -p "Continue with installation? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Installation cancelled."
    exit 0
fi

echo ""
echo "======================================================="
echo "Step 1/4: Installing ROS 2 Humble"
echo "======================================================="
bash "$SCRIPT_DIR/install_ros2_humble.sh"

echo ""
echo "======================================================="
echo "Step 2/4: Installing Dependencies"
echo "======================================================="
bash "$SCRIPT_DIR/install_dependencies.sh"

echo ""
echo "======================================================="
echo "Step 3/4: Setting up USB Permissions"
echo "======================================================="
bash "$SCRIPT_DIR/setup_realsense_udev.sh"

echo ""
echo "======================================================="
echo "Step 4/4: Building Workspace"
echo "======================================================="
bash "$SCRIPT_DIR/build_workspace.sh"

echo ""
echo "======================================================="
echo "Setup Complete!"
echo "======================================================="
echo ""
echo "IMPORTANT NEXT STEPS:"
echo ""
echo "1. REBOOT your system or log out/in for all changes to take effect:"
echo "   sudo reboot"
echo ""
echo "2. After reboot, connect your RealSense D435 camera"
echo ""
echo "3. Test the camera:"
echo "   realsense-viewer"
echo ""
echo "4. Start the scanner:"
echo "   $SCRIPT_DIR/start_scanner.sh"
echo ""
echo "5. (Optional) Setup auto-start on boot:"
echo "   sudo $SCRIPT_DIR/install_service.sh"
echo "   sudo systemctl enable --now room-scanner"
echo ""
echo "For more information, see README.md"
echo ""
