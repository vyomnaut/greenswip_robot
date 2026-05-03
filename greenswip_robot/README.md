# Greenswip Robotics Assignment — ROS 2 Ackermann Perception-to-Action Pipeline

## Overview
Complete ROS 2 perception-to-action pipeline for an Ackermann-steered mobile robot.

- **Simulation**: Gazebo Ignition (shapes.sdf world)
- **Perception**: OpenCV red+shape detection → `/box_detection`
- **Control**: Proportional Ackermann visual-servo → `/cmd_ackermann`

## Dependencies
```bash
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge \
    ros-<humble>-ackermann-msgs ros-<humble>-cv-bridge python3-opencv
```

## Build & Run
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Launch simulation + all nodes
cd ~/ros2_ws && colcon build --packages-select greenswip_robot && source install/setup.bash
ros2 launch greenswip_robot greenswip_sim.launch.py debug_vision:=true

an then 
ros2 run greenswip_robot control_node.py


```

## Topic Map
| Topic | Type | Direction |
|-------|------|-----------|
| `/camera/image_raw` | sensor_msgs/Image | Gazebo → perception_node |
| `/box_detection` | std_msgs/Float32MultiArray [cx_norm, cy_norm, area_norm, flag] | perception → control |
| `/cmd_ackermann` | ackermann_msgs/AckermannDriveStamped | control → Gazebo |
| `/odom` | nav_msgs/Odometry | Gazebo → ROS |
| `/joint_states` | sensor_msgs/JointState | Gazebo → robot_state_publisher |

## Key Design Decisions
- **Dual red mask** (HSV 0–10° ∪ 170–180°) handles hue wrap-around
- **Shape filter** (approxPolyDP 4–6 vertices) distinguishes box from sphere/capsule/cylinder
- **Speed-proportional-to-steer** law respects Ackermann minimum turning radius
- **Arc search** (not spin-in-place) used when target is lost — kinematically valid
