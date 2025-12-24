#!/bin/bash
# Install ROS 2 Humble on Ubuntu 22.04

set -e

echo "============================================"
echo "Installing ROS 2 Humble Hawksbill"
echo "============================================"

# Check if running Ubuntu 22.04
if [ "$(lsb_release -cs)" != "jammy" ]; then
    echo "Error: This script is for Ubuntu 22.04 (Jammy)"
    echo "Your version: $(lsb_release -ds)"
    exit 1
fi

# Set locale
echo "Setting up locale..."
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
echo "Setting up ROS 2 sources..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

# Add ROS 2 GPG key
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
echo "Installing ROS 2 Humble (this may take a while)..."
sudo apt update
sudo apt upgrade -y

# Install base ROS 2
sudo apt install -y ros-humble-desktop

# Install development tools
echo "Installing ROS 2 development tools..."
sudo apt install -y \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Initialize rosdep
echo "Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Setup environment
echo "Setting up environment..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

echo ""
echo "============================================"
echo "ROS 2 Humble installation complete!"
echo "============================================"
echo ""
echo "To activate ROS 2 in current terminal:"
echo "  source /opt/ros/humble/setup.bash"
echo ""
echo "This will be automatically sourced in new terminals."
echo ""
