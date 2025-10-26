# joy_to_twist

## Overview
`joy_to_twist` is a ROS 2 node that converts joystick input (`sensor_msgs/msg/Joy`) into velocity (`geometry_msgs/msg/Twist`) and servo control (`std_msgs/msg/Int32`) commands.  
It is designed for teleoperation of mobile robots or robotic platforms that require simultaneous driving and servo control from a standard joystick.

## Features
- Converts joystick axes to linear and angular velocity commands (`/cmd_vel`)
- Controls up to two servo motors via D-pad (vertical for servo1, horizontal for servo2)
- Configurable angle limits and step increments
- Lightweight and fully compliant with ROS 2 Python (`rclpy`)
- Compatible with standard `joy` package

## Topics

| Topic Name | Direction | Message Type | Description |
|-------------|------------|---------------|--------------|
| `/joy` | Subscribe | `sensor_msgs/msg/Joy` | Joystick input data |
| `/cmd_vel` | Publish | `geometry_msgs/msg/Twist` | Robot velocity command |
| `/servo1/angle_cmd` | Publish | `std_msgs/msg/Int32` | Servo 1 angle command (D-pad up/down) |
| `/servo2/angle_cmd` | Publish | `std_msgs/msg/Int32` | Servo 2 angle command (D-pad left/right) |
| `/servo.../angle_cmd` | Publish | `std_msgs/msg/Int32` | Servo ... angle command (D-pad left/right) |

---

## Node Behavior

- **Left stick horizontal (`axes[0]`)** → Linear velocity (X)
- **Left stick vertical (`axes[1]`)** → Angular velocity (Z)
- **D-pad vertical (`axes[7]`)** → Servo 1 (up/down)
- **D-pad horizontal (`axes[6]`)** → Servo 2 (left/right)

Each D-pad press changes the corresponding servo by ±25° within a range of -150° to +150°.  
Velocity scaling can be adjusted directly in the node source.

---

## Installation

### 1. Clone the package
```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/joy_to_twist.git
```

### 2. Build the package
```bash
cd ~/ros2_ws
colcon build --packages-select joy_to_twist
```

### 3. Source the workspace
```bash
source install/setup.bash
```

---

## Usage

### Run the joy_to_twist node
```bash
ros2 run joy_to_twist joy_to_twist
```

The node will log servo movements and publish velocity commands based on joystick input.

---

## License
This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
