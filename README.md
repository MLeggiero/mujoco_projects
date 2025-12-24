# 3D Room Scanner with Intel RealSense D435

A ROS 2 Humble-based 3D room scanning system with Intel RealSense D435 depth camera, touchscreen visualization, and optional IMU integration.

## System Overview

### Hardware Requirements
- **Laptop/Desktop** or Raspberry Pi 4 (4GB+ RAM minimum, 8GB recommended)
- Intel RealSense D435 depth camera
- Display (touchscreen optional for portable setups)
- External IMU (optional, for future integration)
- Ubuntu 22.04 (64-bit recommended for ROS 2 Humble)

**Note:** Current configuration is optimized for laptop/desktop performance (1280x720@30fps). For Raspberry Pi, see Performance Tuning section below.

### Software Stack
- **ROS 2 Humble Hawksbill**
- **Intel RealSense ROS 2 Wrapper** - Camera interface
- **RTABMap ROS** - Real-time point cloud mapping and SLAM
- **RViz2** - 3D visualization
- **PCL (Point Cloud Library)** - Point cloud processing

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    RealSense D435 Camera                     │
└───────────────────────────┬─────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│              realsense2_camera (ROS 2 Node)                  │
│  Publishes: /camera/color/image_raw                          │
│            /camera/depth/image_rect_raw                      │
│            /camera/depth/color/points                        │
│            /camera/color/camera_info                         │
└──────────┬──────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────┐
│              RTABMap or Octomap Server                       │
│  - Aggregates point clouds over time                         │
│  - Performs SLAM (optional)                                  │
│  - Publishes: /map (point cloud map)                         │
│               /octomap_full (3D occupancy grid)              │
└──────────┬──────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────┐
│                      RViz2 Visualization                     │
│  - Display live point cloud                                  │
│  - Display aggregated map                                    │
│  - Touch-friendly interface                                  │
└─────────────────────────────────────────────────────────────┘
```

## Installation

### 1. Install ROS 2 Humble

```bash
cd ~/mujoco_projects
./scripts/install_ros2_humble.sh
```

### 2. Install Dependencies

```bash
./scripts/install_dependencies.sh
```

### 3. Build Workspace

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Set Up USB Rules for RealSense

```bash
./scripts/setup_realsense_udev.sh
```

## Usage

### Quick Start (Manual)

```bash
# Start the full system (camera + mapping + RViz)
ros2 launch room_scanner scanner.launch.py

# Or without RViz (headless mode)
ros2 launch room_scanner scanner.launch.py use_rviz:=false
```

### Auto-Start on Boot

```bash
# Install systemd service
sudo ./scripts/install_service.sh

# Enable auto-start
sudo systemctl enable room-scanner

# Start now
sudo systemctl start room-scanner
```

### Saving Point Cloud Maps

While running, you can save the current map:

```bash
# Save point cloud to PCD file
ros2 service call /rtabmap/save_map std_srvs/srv/Empty

# Or use octomap save (if using octomap)
ros2 run octomap_server octomap_saver -f ~/maps/scan_$(date +%Y%m%d_%H%M%S).bt
```

### Configuration

Edit configuration files in `config/`:
- `camera_params.yaml` - RealSense camera settings
- `rtabmap_params.yaml` - Mapping parameters
- `octomap_params.yaml` - Octomap settings (alternative)

## Package Structure

```
mujoco_projects/
├── ros2_ws/                    # ROS 2 workspace
│   └── src/
│       └── room_scanner/       # Custom package
│           ├── launch/         # Launch files
│           ├── config/         # Configuration files
│           └── package.xml
├── scripts/                    # Installation scripts
├── rviz/                       # RViz configurations
└── config/                     # Additional configs
```

## ROS 2 Topics

### Published by RealSense
- `/camera/color/image_raw` - RGB image
- `/camera/depth/image_rect_raw` - Depth image
- `/camera/depth/color/points` - Point cloud (colored)
- `/camera/color/camera_info` - Camera calibration
- `/camera/depth/camera_info` - Depth camera calibration

### Published by Mapping Node
- `/rtabmap/mapData` - RTAB-Map data
- `/rtabmap/cloud_map` - Aggregated point cloud
- `/octomap_full` - Full 3D occupancy map (if using octomap)

## Performance Tuning

### Current Configuration (Laptop/Desktop)
The configuration files are set for high-quality scanning:
- High image resolution (1280x720)
- 30 FPS capture rate
- Minimal point cloud decimation (cloud_decimation: 1)
- Fine voxel grid (5mm)
- More features for better tracking (1000 features)

### For Raspberry Pi 4
If deploying on Raspberry Pi, reduce these settings in the config files:

**In `config/camera_params.yaml`:**
```yaml
rgb_camera.profile: '640x480x15'      # Reduce from 1280x720x30
depth_module.profile: '640x480x15'    # Reduce from 1280x720x30
tf_publish_rate: 10.0                 # Reduce from 30.0
```

**In `config/rtabmap_params.yaml`:**
```yaml
queue_size: 10                        # Reduce from 30
detection_rate: 1.0                   # Reduce from 2.0
cloud_decimation: 2                   # Increase from 1
cloud_voxel_size: 0.01                # Increase from 0.005
Vis/MaxFeatures: '400'                # Reduce from '1000'
Optimizer/Iterations: '10'            # Reduce from '20'
```

After editing, rebuild the workspace:
```bash
cd ros2_ws && colcon build --symlink-install
```

## Future Enhancements

### IMU Integration
When adding an external IMU:
1. Install IMU ROS 2 driver (e.g., `imu_filter_madgwick`)
2. Update launch file to include IMU node
3. Configure RTABMap to fuse IMU data for better odometry

### Advanced Features
- **Loop Closure** - Already enabled in RTABMap for better mapping
- **Mesh Generation** - Add mesh_tools or Open3D integration
- **Export to other formats** - PLY, OBJ, STL for 3D printing

## Troubleshooting

### Camera not detected
```bash
# Check USB connection
lsusb | grep Intel

# Check RealSense firmware
rs-fw-update -l
```

### Performance issues
- Reduce resolution in `config/camera_params.yaml`
- Lower frame rate
- Disable RGB processing if only depth needed
- Increase point cloud voxel size

### RViz2 crashes or slow
- Use the lightweight config: `rviz/scanner_minimal.rviz`
- Disable grid and other heavy visualizations
- Reduce point cloud display size

## Resources

- [RealSense ROS 2 Wrapper](https://github.com/IntelRealSense/realsense-ros)
- [RTABMap ROS](https://github.com/introlab/rtabmap_ros)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)

## License

See LICENSE file.
