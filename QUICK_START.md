# Quick Start Guide

## First Time Setup

```bash
# Run complete setup (installs everything)
./scripts/complete_setup.sh

# Reboot after installation
sudo reboot
```

## Testing the Camera

```bash
# Test RealSense camera (GUI viewer)
realsense-viewer

# List available ROS topics
ros2 topic list

# View point cloud data
ros2 topic echo /camera/depth/color/points --once
```

## Running the Scanner

### Manual Start
```bash
# Start the full system (camera + mapping + RViz)
./scripts/start_scanner.sh

# Or manually:
cd ros2_ws
source install/setup.bash
ros2 launch room_scanner scanner.launch.py
```

### Camera Only (for testing)
```bash
cd ros2_ws
source install/setup.bash
ros2 launch room_scanner camera_only.launch.py
```

### Custom Options
```bash
# Without RViz (headless)
ros2 launch room_scanner scanner.launch.py use_rviz:=false

# Use Octomap instead of RTABMap
ros2 launch room_scanner scanner.launch.py use_rtabmap:=false use_octomap:=true

# Minimal RViz config (better performance)
rviz2 -d $(ros2 pkg prefix room_scanner)/share/room_scanner/rviz/scanner_minimal.rviz
```

## Auto-Start on Boot

```bash
# Install systemd service
sudo ./scripts/install_service.sh

# Enable and start
sudo systemctl enable --now room-scanner

# Check status
sudo systemctl status room-scanner

# View logs
journalctl -u room-scanner -f
```

## Saving Maps

### RTABMap Database
The map is continuously saved to `~/.ros/rtabmap.db`

To export point cloud:
```bash
# While RTABMap is running
ros2 service call /rtabmap/save_map std_srvs/srv/Empty

# Or use rtabmap-databaseViewer (after stopping RTABMap)
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

### Octomap
```bash
# Save current octomap
ros2 run octomap_server octomap_saver -f ~/maps/scan_$(date +%Y%m%d_%H%M%S).bt

# Convert to other formats
# .bt to .ot (full octree)
# .bt to .pcd (point cloud)
```

### Point Cloud (PCD)
```bash
# Record point cloud topic
ros2 bag record /camera/depth/color/points

# Or subscribe and save with PCL tools
ros2 run pcl_ros pointcloud_to_pcd input:=/camera/depth/color/points
```

## Troubleshooting

### Camera not detected
```bash
# List USB devices
lsusb | grep Intel

# Test with RealSense tools
rs-enumerate-devices

# Check permissions
ls -l /dev/video*
```

### Performance issues on Raspberry Pi
**Note:** Current config is optimized for laptop/desktop (1280x720@30fps). For Raspberry Pi:

1. Edit `ros2_ws/src/room_scanner/config/camera_params.yaml`:
   - Reduce resolution: `'1280x720x30'` → `'640x480x15'`
   - Lower tf_publish_rate: `30.0` → `10.0`

2. Edit `ros2_ws/src/room_scanner/config/rtabmap_params.yaml`:
   - Reduce queue_size: `30` → `10`
   - Reduce detection_rate: `2.0` → `1.0` (process every second)
   - Increase cloud_decimation: `1` → `2`
   - Increase cloud_voxel_size: `0.005` → `0.01`
   - Reduce Vis/MaxFeatures: `'1000'` → `'400'`

3. Rebuild workspace:
   ```bash
   cd ros2_ws
   colcon build --symlink-install
   ```

### RViz is slow
```bash
# Use minimal config
rviz2 -d $(ros2 pkg prefix room_scanner)/share/room_scanner/rviz/scanner_minimal.rviz

# Or disable RViz entirely
ros2 launch room_scanner scanner.launch.py use_rviz:=false
```

## Useful Commands

```bash
# View all topics
ros2 topic list

# Monitor topic frequency
ros2 topic hz /camera/color/image_raw

# View TF tree
ros2 run tf2_tools view_frames

# Record data
ros2 bag record -a  # Record all topics
ros2 bag record /camera/color/image_raw /camera/depth/image_rect_raw

# Reset RTABMap database
ros2 service call /rtabmap/reset_map std_srvs/srv/Empty
```

## Configuration Files

- Camera settings: `ros2_ws/src/room_scanner/config/camera_params.yaml`
- RTABMap settings: `ros2_ws/src/room_scanner/config/rtabmap_params.yaml`
- Octomap settings: `ros2_ws/src/room_scanner/config/octomap_params.yaml`
- RViz layout: `rviz/scanner_config.rviz`

## Future: Adding IMU

When you add an external IMU:

1. Install IMU driver (example for MPU6050):
```bash
sudo apt install ros-humble-imu-tools
```

2. Update launch file to include IMU node

3. Configure RTABMap to use IMU data:
   - Edit `config/rtabmap_params.yaml`
   - Add: `subscribe_imu: true`
   - Set: `Idom/Strategy: '1'` (use IMU for odometry)

4. Remap IMU topic in launch file:
```python
remappings=[
    ('imu', '/imu/data'),
]
```

## Resources

- [Full Documentation](README.md)
- [RealSense ROS Wrapper](https://github.com/IntelRealSense/realsense-ros)
- [RTABMap Documentation](http://wiki.ros.org/rtabmap_ros)
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
