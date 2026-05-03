# Greenswip Ackermann Robot - ROS2 Perception to Action Pipeline

## What it does
- Detects red box using OpenCV HSV masking
- Navigates using Ackermann steering (no spin in place)
- Runs in Gazebo Ignition simulation

## Run
```bash
ros2 launch greenswip_robot greenswip_sim.launch.py debug_vision:=true
ros2 run greenswip_robot control_node.py
```<img width="1920" height="1080" alt="image" src="https://github.com/user-attachments/assets/9b86c12d-e2cd-4c29-9952-c5858776cb08" />
