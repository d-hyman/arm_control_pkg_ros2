# arm_control_pkg

### `sorter_node` — the decision-maker

Subscribes to:
- `detections` (`vision_msgs/Detection2DArray`) — object class, confidence, and bounding box, published by the vision node (or `fake_detector_node` while it isn't ready yet)
- `manual_sort` (`std_msgs/String`) — send `"left"` or `"right"` to manually trigger a sort, for testing without a camera

Publishes:
- `joint_commands` (`sensor_msgs/JointState`, radians) — the arm's target joint angles, published continuously at 20 Hz
- `sorter_status` (`std_msgs/String`) — the current step, e.g. `"above pick"`, `"idle"`, or `"bottle->left"`

### `servo_driver_node` — talks to the hardware

Subscribes to `joint_commands` 

There are 3 aspects to the `mode` parameter:
- **`sim`** (default) — angles only update the RViz model; no hardware involved.
- **`serial`** — angles are sent as an ASCII line over USB (`serial_port`, `baud_rate` params) to a microcontroller. If the port fails to open, it logs an error and falls back to `sim` automatically.
- **`http`** — *planned, not yet implemented.* Will send angles over WiFi to a Freenove ESP32, which moves the real servos through a PCA9685.

### `fake_detector_node` — pretends to be the AI

Vision package placeholder if such as package is not connected to the ros2 system. It cycles through a list of classes, publishing each on `detections` (`vision_msgs/Detection2DArray`) for a few seconds at a time.

When implementing a vison package, turn off with `fake_detector:=false`.

## Launch

`launch/arm_control.launch.py` uses the RViz2 arm model from the package `robot_arm_assem_v5`),

Launch arguments:
- `fake_detector` - `true` or `false`
- `driver_mode` - `sim` for RViz only, `serial` for real hardware over USB
- `rviz`

## How to Run

**Prerequisites:** Ubuntu (or another Linux distro) with ROS 2 installed and sourced.

1. **Install ROS 2** (if not already installed)
   Follow the official instructions for your distro: https://docs.ros.org/en/humble/Installation.html

2. **Source your ROS 2 installation** (add this to `~/.bashrc` to avoid repeating it every terminal session)
   ```bash
   source /opt/ros/<your-ros-distro>/setup.bash
   ```

3. **Build and run**
   ```bash
   colcon build --packages-select arm_control_pkg robot_arm_assem_v5
   source install/setup.bash
   ros2 launch arm_control_pkg arm_control.launch.py
   ```