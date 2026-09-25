### How to Run

**Prerequisites:** Ubuntu (or another Linux distro) with ROS 2 installed and sourced.

1. **Install ROS 2** (if not already installed)  
   Follow the official instructions for your distro: https://docs.ros.org/en/humble/Installation.html

2. **Source your ROS 2 installation** (add this to `~/.bashrc` to avoid repeating it every terminal session)
```bash
   source /opt/ros/<your-ros-distro>/setup.bash
```
```bash

colcon build --packages-select arm_control_pkg robot_arm_assem_v5
source install/setup.bash
ros2 launch arm_control_pkg arm_control.launch.py
```

### Note
This ros2 package uses fake_detector_node as a placeholder for a seperate AI vision package which uses yolo to detect objects