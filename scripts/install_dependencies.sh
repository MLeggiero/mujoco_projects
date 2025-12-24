#!/bin/bash
# Install all dependencies for the 3D Room Scanner

set -e

echo "============================================"
echo "Installing 3D Room Scanner Dependencies"
echo "============================================"

# Source ROS 2 if available
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS 2 Humble not found. Please run install_ros2_humble.sh first."
    exit 1
fi

# Update package list
echo "Updating package list..."
sudo apt update

# Install Intel RealSense SDK
echo ""
echo "Installing Intel RealSense SDK..."
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

sudo apt install -y \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg

# Install RealSense ROS 2 wrapper
echo ""
echo "Installing RealSense ROS 2 packages..."
sudo apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-camera-msgs \
    ros-humble-realsense2-description

# Install RTABMap
echo ""
echo "Installing RTABMap..."
sudo apt install -y \
    ros-humble-rtabmap-ros \
    ros-humble-rtabmap-rviz-plugins

# Install Octomap (alternative to RTABMap)
echo ""
echo "Installing Octomap..."
sudo apt install -y \
    ros-humble-octomap-server \
    ros-humble-octomap-msgs \
    ros-humble-octomap-rviz-plugins

# Install Point Cloud Library
echo ""
echo "Installing PCL and related tools..."
sudo apt install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    libpcl-dev \
    pcl-tools

# Install additional ROS 2 packages
echo ""
echo "Installing additional ROS 2 packages..."
sudo apt install -y \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
sudo apt install -y \
    python3-pip \
    python3-numpy \
    python3-opencv

pip3 install --user \
    opencv-python \
    numpy

# Install build tools
echo ""
echo "Installing build tools..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget

# Install USB permissions tools
echo ""
echo "Installing USB tools..."
sudo apt install -y \
    udev \
    libusb-1.0-0-dev

echo ""
echo "============================================"
echo "Dependency installation complete!"
echo "============================================"
echo ""
echo "Verifying RealSense installation..."
if command -v realsense-viewer &> /dev/null; then
    echo "✓ RealSense SDK installed successfully"
    echo "  You can test the camera with: realsense-viewer"
else
    echo "⚠ Warning: realsense-viewer not found in PATH"
fi

echo ""
echo "Next steps:"
echo "1. Run setup_realsense_udev.sh to configure USB permissions"
echo "2. Build the workspace with: cd ros2_ws && colcon build"
echo "3. Connect your RealSense camera and test with: realsense-viewer"
echo ""
