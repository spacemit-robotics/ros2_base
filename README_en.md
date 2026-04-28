# ESOS Base Control

## Project Overview

This component is a ROS 2-based chassis control node used to communicate with a small-core differential-drive chassis driver over RPMsg, enabling velocity control and odometry publishing for a mobile robot base.

It mainly addresses the following needs:

- Converts the ROS 2 standard velocity command `/cmd_vel` into motion control commands executable by the chassis;
- Retrieves pose and velocity information from the chassis side and publishes standard `/odom` odometry messages;
- Optionally publishes the `odom -> base_footprint` TF transform, making it easy for upper-layer navigation, localization, mapping, and related modules to integrate directly;
- Provides configurable parameters such as wheel diameter, wheel base, motor correction factors, and RPMsg device settings to adapt to different robot hardware configurations.

This component is suitable as the ROS 2 mobile robot chassis integration layer and can be easily integrated with navigation, teleoperation, voice control, SLAM, and other systems.

## Features

### Supported Features

- Subscribes to `/cmd_vel` velocity control commands of type `geometry_msgs/msg/Twist`;
- Publishes `/odom` odometry messages of type `nav_msgs/msg/Odometry`;
- Publishes the TF transform from `odom` to `base_footprint`;
- Supports a two-wheel differential-drive chassis model;
- Supports parameter-based configuration for:
  - Chassis wheel diameter: `wheel_diameter`
  - Wheel base: `wheel_base`
  - Left/right motor correction factors: `motor1_factor` / `motor2_factor`
  - Frame and topic names for odometry/TF
  - RPMsg control and data channel parameters
- Supports timeout protection: if no `/cmd_vel` command is received for a period of time, the node automatically sends zero velocity to reduce the risk of uncontrolled motion;
- Relies on the `chassis` capability and `drv_rpmsg_esos` driver in `components/control/base` for hardware communication.

### Not Supported Yet / Notes

- The current implementation is intended for two-wheel differential-drive chassis only, and is not suitable for omnidirectional, mecanum, or Ackermann chassis;
- Wheel diameter, wheel base, and motor factors must be tuned according to the actual hardware;
- The node depends on the underlying chassis driver and RPMsg channels working correctly by default. If the low-level components are not built or deployed properly, the node cannot start.

## Quick Start

Below is the shortest path from zero to a runnable setup, suitable for first-time integration and quick verification.

### Environment Preparation

Before getting started, make sure the following prerequisites are met:

1. ROS 2 has been installed and configured;
2. The underlying chassis library `components/control/base` has been built and installed correctly;
3. The RPMsg device node or service available on the system is ready to use;
4. The target hardware is a small-core-controlled differential-drive chassis.

This component depends on the following ROS 2 packages:

- `rclcpp`
- `geometry_msgs`
- `nav_msgs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`

If you build from source in a workspace, make sure the artifacts from `components/control/base` have already been installed into a path discoverable by `CMAKE_PREFIX_PATH`; otherwise, compilation will fail with an error such as:

```text
chassis.h or libchassis not found. Build/install components/control/base first
```

### Build

After entering the ROS 2 workspace, run:

```bash
colcon build --packages-select base
```

If the environment has not yet been loaded in the current workspace, run the following after the build completes:

```bash
source install/setup.bash
```

> Notes:
>
> - The package name is `base`;
> - The executable is `esos_base_control_node`;
> - The launch file is `esos_base_control.launch.py`.

### Run Examples

#### 1. Run the node directly

```bash
ros2 run base esos_base_control_node
```

#### 2. Launch with the launch file

```bash
ros2 launch base esos_base_control.launch.py
```

#### 3. Launch with parameters

```bash
ros2 launch base esos_base_control.launch.py \
    wheel_diameter:=0.067 \
    wheel_base:=0.28
```

#### 4. Send a test velocity command

After the node starts, you can publish a test velocity command with:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

#### 5. Check odometry output

```bash
ros2 topic echo /odom
```

If TF publishing is enabled, you can also use tools such as `rviz2` and `tf2_tools` to verify whether the transforms are correct.

## Detailed Usage

For now, you can focus on the following key parameters:

| Parameter | Default | Description |
|---|---:|---|
| `send_hz` | `20.0` | Velocity command send frequency (Hz) |
| `odom_hz` | `50.0` | Odometry publish frequency (Hz) |
| `cmd_vel_timeout` | `0.4` | Control command timeout (seconds) |
| `publish_tf` | `true` | Whether to publish TF |
| `odom_topic` | `odom` | Odometry topic name |
| `odom_frame` | `odom` | Odometry frame |
| `base_frame` | `base_footprint` | Robot base frame |
| `wheel_diameter` | `0.067` | Wheel diameter in meters |
| `wheel_base` | `0.28` | Wheel base in meters |
| `motor1_factor` | `1.0` | Left wheel velocity correction factor |
| `motor2_factor` | `1.0` | Right wheel velocity correction factor |
| `rpmsg_ctrl_dev` | `""` | RPMsg control device; empty means use the default |
| `rpmsg_data_dev` | `""` | RPMsg data device; empty means use the default |
| `rpmsg_service_name` | `""` | RPMsg service name; empty means use the default |
| `rpmsg_local_addr` | `0` | RPMsg local address; `0` means default |
| `rpmsg_remote_addr` | `0` | RPMsg remote address; `0` means default |

## FAQ

### 1. Compilation fails with `chassis.h` or `libchassis` not found

This is usually because the underlying `components/control/base` has not been built and installed yet, or its install path has not been added to `CMAKE_PREFIX_PATH`.

Recommended checks:

- Confirm that the underlying library has been compiled and installed successfully;
- Confirm that the current shell has sourced the corresponding environment setup script;
- Check whether `CMAKE_PREFIX_PATH` contains the install path of the underlying library.

### 2. The node starts, but the chassis does not move

Recommended checks:

- Verify that `/cmd_vel` is actually receiving messages;
- Check whether `cmd_vel_timeout` is too small, causing the velocity to be reset to zero shortly after sending;
- Check whether the RPMsg device nodes exist and are accessible;
- Check whether the ESOS small-core firmware and driver are working properly;
- Check whether the left/right motor correction factors are set incorrectly.

### 3. `/odom` is published, but the data is abnormal or drifts significantly

Key items to verify:

- Whether `wheel_diameter` matches the physical wheel size;
- Whether `wheel_base` has been measured accurately;
- Whether the left/right motor correction factors need calibration;
- Whether floor friction, wheel slip, encoder precision, or other physical factors are affecting the result.

### 4. TF conflicts or inconsistent frames

If other modules in the system are already publishing `odom -> base_footprint` or similar TF transforms, conflicts may occur.

Recommendations:

- Clearly define a single TF publisher source for the full robot system;
- If necessary, set `publish_tf` to `false` and publish the transform from an upper-layer module instead;
- Check whether `odom_frame` and `base_frame` are consistent with the robot-wide frame convention.

### 5. Velocity direction is incorrect or turning direction is reversed

This is usually related to chassis wiring, motor direction definitions, left/right wheel mapping, or low-level driver configuration.

It is recommended to start with low-speed tests:

- Send forward velocity only and verify whether the robot moves straight;
- Send angular velocity only and verify whether the rotation direction matches expectations;
- If there is yaw drift or deviation, fine-tune `motor1_factor` and `motor2_factor`.

## Contributing

Contributions are welcome through issues, documentation revisions, code patches, test feedback, and other forms.

Suggested contribution workflow:

1. Read the relevant development guidelines and directory descriptions in this repository;
2. Confirm the scope of impact before making changes to avoid breaking existing interfaces or behavior;
3. Complete the necessary build and basic functional verification before submitting changes;
4. Add corresponding documentation for any new parameters, interfaces, or behavior changes.

## License

The source file headers of this component declare Apache-2.0. The final license terms are subject to the `LICENSE` file in this directory.
