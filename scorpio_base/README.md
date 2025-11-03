# Scorpio Base Driver (ROS2)

Hardware driver for the Scorpio robot base, ported from ROS1 to ROS2 Humble.

## Features

- **Modern C++17** implementation with composition support
- **Zero boost dependencies** - uses standard library (`std::thread`, `std::mutex`, `std::function`)
- **Integrated cereal_port** - serial communication library embedded into the package
- **Two nodes**:
  - `scorpio_base_node`: Main hardware driver (IMU + Odometry + Motor control)
  - `cmd_vel_to_ackermann_node`: Twist to Ackermann converter

## Hardware Requirements

- Scorpio robot base with:
  - STM32 controller on `/dev/ttyS0` @ 115200 baud
  - Motor controller on `/dev/ttyS3` @ 57600 baud
- IMU sensor
- Hall encoder (optional)

## Installation

```bash
cd ~/ros2_ws
colcon build --packages-select scorpio_base
```

## Usage

### Launch the driver

```bash
ros2 launch scorpio_base scorpio_base_launch.py
```

### Custom parameters

```bash
ros2 launch scorpio_base scorpio_base_launch.py \
  stm32_port:=/dev/ttyUSB0 \
  motor_port:=/dev/ttyUSB1 \
  limited_speed:=0.8 \
  wheelbase:=0.315
```

### Running nodes separately

```bash
# Main driver
ros2 run scorpio_base scorpio_base_node

# Converter
ros2 run scorpio_base cmd_vel_to_ackermann_node
```

## Topics

### Subscribed

- `~/ackermann_cmd` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))
  - Ackermann drive commands (speed + steering angle)
- `cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))
  - Twist commands (converted to Ackermann by `cmd_vel_to_ackermann`)

### Published

- `imu_data` ([sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html))
  - IMU data from STM32
- `odom` ([nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))
  - Odometry from hall encoder (if enabled)
- `feedback_velocity` ([geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))
  - Actual velocity feedback
- `ackermann_cmd` (from `cmd_vel_to_ackermann`)

## Parameters

### scorpio_base_node

| Parameter       | Type   | Default           | Description                    |
|----------------|--------|-------------------|--------------------------------|
| `base_frame_id`| string | `base_footprint`  | Base frame ID                  |
| `odom_frame_id`| string | `odom`            | Odometry frame ID              |
| `stm32_port`   | string | `/dev/ttyS0`      | STM32 serial port              |
| `motor_port`   | string | `/dev/ttyS3`      | Motor controller serial port   |
| `stm32_baud`   | int    | `115200`          | STM32 baud rate                |
| `motor_baud`   | int    | `57600`           | Motor baud rate                |
| `hall_encoder` | bool   | `true`            | Enable hall encoder odometry   |
| `limited_speed`| double | `1.0`             | Maximum speed (m/s)            |
| `wheelbase`    | double | `0.315`           | Wheelbase (m)                  |

### cmd_vel_to_ackermann_node

| Parameter    | Type   | Default | Description       |
|-------------|--------|---------|-------------------|
| `frame_id`  | string | `odom`  | Frame ID          |
| `wheelbase` | double | `0.315` | Wheelbase (m)     |

## Serial Port Permissions

Add udev rules for serial port access:

```bash
sudo usermod -a -G dialout $USER
# Or create udev rules (see rules/70-ttyusb.rules)
```

## Code Quality

- **clang-format**: Google style with custom tweaks
- **clang-tidy**: Performance + modernize checks
- All tests pass ✅

## Migration from ROS1

Key changes from original `scorpio_base_ros1`:

1. **No boost dependencies** - replaced with STL
2. **Composition-based** - can be loaded into same process
3. **Modern C++** - smart pointers, RAII, STL algorithms
4. **Removed goto** - replaced with structured loops
5. **Better error handling** - exceptions with RCLCPP logging
6. **Type safety** - explicit casts, const correctness

## Credits

- Original author: Litian Zhuang (NXROBO)
- ROS2 port: Lihan Chen (2025)

## License

Apache-2.0
