# Obstacle Avoidance Simulation - Quick Start Guide

## Prerequisites Check

```bash
# Make sure you have Gazebo and ROS2 tools installed
apt-cache policy gazebo-common
apt-cache policy ros-humble-rviz2
apt-cache policy ros-humble-gazebo-ros

# If any are missing, install:
sudo apt-get install -y gazebo gazebo-common ros-humble-rviz2 ros-humble-gazebo-ros
```

## Option 1: Quick Start (Bash Script)

```bash
cd ~/ros2_ws

# Build the package
colcon build --packages-select obstacle_avoidance

# Run simulation (auto-starts Gazebo + RViz + Avoider)
bash src/obstacle_avoidance/run_simulation.sh

# Or with custom parameters:
bash src/obstacle_avoidance/run_simulation.sh 0.6 0.4
#                                        ^^^   ^^^
#                                safe_dist  forward_speed
```

## Option 2: Manual - More Control

### Terminal 1: Start Gazebo

```bash
cd ~/ros2_ws
source install/setup.bash
gazebo ~/ros2_ws/src/obstacle_avoidance/worlds/obstacle_world.world
```

### Terminal 2: Start RViz (Visualization)

```bash
cd ~/ros2_ws
source install/setup.bash
rviz2 -d ~/ros2_ws/src/obstacle_avoidance/rviz/simulation.rviz
```

### Terminal 3: Run Obstacle Avoidance

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run obstacle_avoidance avoider --ros-args \
  -p safe_distance:=0.8 \
  -p forward_speed:=0.3 \
  -p turn_speed:=1.5
```

### Terminal 4: Monitor (Optional)

```bash
cd ~/ros2_ws
source install/setup.bash

# Watch sensor data
ros2 topic echo /scan

# Or watch commands sent to robot
ros2 topic echo /cmd_vel

# Or see real-time values
ros2 topic hz /scan  # How fast lidar updates
ros2 topic hz /cmd_vel  # How fast we send commands
```

---

## What You Should See

1. **Gazebo Window:**
   - Blue box robot in center
   - Red walls on left/right
   - Yellow + Green obstacles scattered around
   - Robot starts moving forward

2. **RViz Window:**
   - Green circle = laser scan data
   - Red obstacle detection visualization
   - Robot odometry (white path)

3. **Terminal Output:**
   ```
   [ObstacleAvoider] State: MOVING_FORWARD | Front: 4.50m | Cmd: linear=0.30, angular=0.00
   [ObstacleAvoider] State: OBSTACLE_DETECTED | Front: 0.75m | Cmd: linear=0.00, angular=0.00
   [ObstacleAvoider] State: TURNING | Front: 0.75m | Cmd: linear=0.00, angular=1.50
   [ObstacleAvoider] State: MOVING_FORWARD | Front: 1.80m | Cmd: linear=0.30, angular=0.00
   ```

---

## Common Issues

### "gazebo: command not found"
```bash
sudo apt-get install gazebo gazebo-common
```

### "Module 'gazebo_ros' not found"
```bash
sudo apt-get install ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs
```

### RViz shows blank screen
- Click on "Add" button in RViz
- Add > By topic > /scan
- Set Fixed Frame to "odom" (top left)

### Robot doesn't move
- Check Terminal 4 output: `ros2 topic hz /cmd_vel`
- If no data flowing, check obstacles vs safe_distance parameter
- Reduce safe_distance: `ros2 param set /obstacle_avoider safe_distance 0.5`

### Gazebo crashes
- Close all Gazebo windows first: `killall gzserver gzclient`
- Try again with verbose output: `gazebo --verbose <worldfile>`

---

## Try Different Approaches

Once basic avoidance works, test the other implementations:

```bash
# Finite State Machine approach
ros2 run obstacle_avoidance fsm_avoider --ros-args -p safe_distance:=0.8

# Behavior-based approach  
ros2 run obstacle_avoidance behavior_based_avoider

# Reactive controller approach
ros2 run obstacle_avoidance reactive_avoider
```

---

## Modify Simulation

Edit the world file to add/remove obstacles:

```bash
nano ~/ros2_ws/src/obstacle_avoidance/worlds/obstacle_world.world
```

Then restart Gazebo to see changes.

---

## Performance Tuning

Watch the logs while robot runs, then adjust:

```bash
# Make it more cautious (stop further from obstacles)
ros2 run obstacle_avoidance avoider --ros-args -p safe_distance:=1.2

# Make it faster
ros2 run obstacle_avoidance avoider --ros-args -p forward_speed:=0.5

# Make it turn faster
ros2 run obstacle_avoidance avoider --ros-args -p turn_speed:=2.0
```

---

## Next Steps

1. ✅ Run simulation with your improved avoider
2. ✅ Try the FSM and behavior-based implementations
3. ✅ Adjust parameters to see how they affect behavior
4. ✅ Once satisfied, deploy to real robot (GO2)

