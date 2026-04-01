# goal
create drone simulator in gazebo simulator. with lidar sensor on top of it.
please use the sjtu_drone as the drone model and add a lidar on it.

## spec
- ROS 2 Humble + Gazebo Classic 11
- sjtu_drone model
- 2D LiDAR flat on top → publishes `/scan` (LaserScan)
- Empty world
- Joystick control via `joy` + `teleop_twist_joy`

## setup

### 1. Clone sjtu_drone (ROS 2 fork) into this workspace
```bash
cd /home/yogev/Desktop/ScanDroneX
git clone https://github.com/NovoG93/sjtu_drone.git
```

### 2. Install dependencies
```bash
cd /home/yogev/Desktop/ScanDroneX
rosdep install --from-paths . --ignore-src -r -y
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy -y
```

### 3. Build
```bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Launch
```bash
ros2 launch scandronex_sim scandronex_sim.launch.py
```

### 5. Check LiDAR topic
```bash
ros2 topic echo /scan
```

### notes
- Hold **RB (R1)** on joystick to enable movement (dead-man switch)
- Hold **LB (L1)** for turbo speed
- If your joystick axis IDs differ, edit `scandronex_sim/config/joy_teleop.yaml`
- If sjtu_drone uses a different cmd_vel topic, edit the remap in `scandronex_sim/launch/scandronex_sim.launch.py`

## Q&A
ROS version: ros2 humble
LiDAR type: 2D LiDAR
Output topics: /scan (LaserScan)
World: empty world
Flight control: joystick (joy + teleop_twist_joy)
