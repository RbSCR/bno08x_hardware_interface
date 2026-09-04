# bno08x_hardware_interface

![Project Status](https://img.shields.io/badge/Status-Work%20In%20Progress-orange)
![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20(Ubuntu%2024.04)-blue?style=flat&logo=ros&logoSize=auto)
![C++](https://img.shields.io/badge/C++-17-blue?style=flat&logo=cplusplus&logoColor=white)
![License](https://img.shields.io/github/license/adityakamath/sts_hardware_interface?label=License)

## Overview

A `ros2_control` `SensorInterface` plugin for the CEVA BNO08x 9-DOF IMU's over I2C.

The BNO08x family (BNO085/BNO086) is a compact System in Package (SiP) with integrated accelerometer, gyroscope, magnetometer, and a 32-bit ARM® Cortex™-M0+ running CEVA's SH-2 firmware. It delivers real-time 3D orientation, heading, calibrated acceleration, and angular velocity, with on-board sensor fusion algorithms and calibration. It supports I2C, SPI, and UART interfaces for sensor data output.

>[!NOTE]
>This plugin only supports I2C.

**Status:** Tested and validated on Raspberry Pi 5 running ROS 2 Kilted (Ubuntu 24.04, aarch64) with real BNO085 hardware.

## Features

- **10 Orientation State Interfaces**: Orientation quaternion (x, y, z, w), angular velocity (rad/s), and linear acceleration (m/s²) — fully compatible with `imu_sensor_broadcaster`
- **Axis Remapping**: 24 standard mounting orientations, configurable at launch, matching BNO08X datasheet §4 Figure 4-3
- **Mock Mode**: Run the complete `ros2_control` lifecycle and publish zero/identity values without any hardware

Optional:

- **3 Magnetometer State Interfaces**: Magnetic field (Tesla) — fully compatible with `magnetometer_broadcaster`
- **Magnetometer Broadcasting**: `ros2 control` `magnetometer_broadcaster`
- **TF Broadcasting**: `imu_tf_broadcaster` relay node republishes the orientation quaternion as a dynamic `world → base_link` TF transform

## Hardware parameters and state interfaces

### Hardware parameters

| Parameter | Type | Default | Description |
| --------- | ---- | ------- | ----------- |
| `i2c_bus` | `int` | `1` | I2C bus number ( plugin opens /dev/i2c-{x} ) |
| `i2c_address` | `string` | `"4A"` | I2C address as hex without 0x prefix ( 4A = 0x4A ) |
| `axis_remap` | `string` | `"East-North-Up"` | Sensor axis placement configuration, see datasheet Figure 4-3  page 41 |
| `imu_rate` | `int` | `100` | Rate at which to measure IMU data (Hz). |
| `enable_magnetometer` | `bool` | `false` | Enable measuring of magnetic field data. |
| `magnetometer_rate` | `int` | `100` | Rate at which to measure magnetic field data (Hz). |
| `enable_mock_mode` | `bool` | `false` | Skip I2C initialisation; publishes identity quaternion and zero velocity/acceleration/magnetic_field |

### State interfaces

| Interface | Unit | Notes |
| --------- | ---- | ----- |
| `orientation.x` | – | Quaternion X |
| `orientation.y` | – | Quaternion Y |
| `orientation.z` | – | Quaternion Z |
| `orientation.w` | – | Quaternion W |
| `angular_velocity.x` | rad/s | Gyroscope X |
| `angular_velocity.y` | rad/s | Gyroscope Y |
| `angular_velocity.z` | rad/s | Gyroscope Z |
| `linear_acceleration.x` | m/s² | Accelerometer X |
| `linear_acceleration.y` | m/s² | Accelerometer Y |
| `linear_acceleration.z` | m/s² | Accelerometer Z |
| `magnetic_field.x` | Tesla | Magnetometer X - when magnetometer enabled |
| `magnetic_field.y` | Tesla | Magnetometer Y - when magnetometer enabled |
| `magnetic_field.z` | Tesla | Magnetometer Z - when magnetometer enabled |

## Launch files and parameters

### Launch files

This package has 3 (example) launch-files:

- `bno08x.launch.py`
- `bno08x_magnetometer.launch.py`
- `bno08x_fixedhwparams.launch.py`

#### bno08x

This file default launches the BNO08x with only the IMU enabled.
The related urdf-file is `bno08x.urdf.xacro` and the related controller-config-file
is `imu_broadcaster.yaml`.

The urdf- and config-file are not configured for the magnetometer.
The urdf-file contains defaults for the hardware parameters, which can be overruled by the
parameters in the launch-file.

#### bno08x_magnetometer

This file default launches the BNO08x with the IMU and the magnetometer enabled.
The related urdf-file is `bno08x_magnetometer.urdf.xacro` and the related controller-config-file
is `imu_magnetometer_broadcaster.yaml`.

The urdf- and config-file are configured for the magnetometer.
The urdf-file contains defaults for the hardware parameters, which can be overruled by the
parameters in the launch-file.

#### bno08x_fixedhwparams

This file default launches the BNO08x with only the IMU enabled.
The related urdf-file is `bno08x_fixedhwparams.urdf.xacro` and the related controller-config-file
is `imu_broadcaster.yaml`.

The urdf- and config-file are not configured for the magnetometer.
The urdf-file contains the values of the hardware parameters, the related "launch-parameters" have
been removed from the launch-file.

This launch-file, urdf-file and controller-config-file can be used as a starting point for an
actual robot.

Note: because this launch-file and urdf-file are also used in a test, the hardware
parameter `enable_moch_mode` does not have a 'fixed' value in the urdf-file, but is still used as
a parameter from the launch-file.

### Launch parameters

The launch parameters enable additional publishers/broadcasters.

The IMU measurements (orientation, angular velocity and linear acceleration) are always broadcasted using the imu_sensor_broadcaster.

| Parameter | Type | Default | Description |
| --------- | ---- | ------- | ----------- |
| `publish_tf` | `bool` | `"true"` | Publish a dynamic world→base_link TF from IMU orientation for RViz visualization |
| `broadcast_magnetometer` | `bool` | `"true"` | Broadcast magnetometer measurements using the magnetometer_broadcaster. To be usefull also set enable_magnetometer to true |

The hardware parameters -[see the hardware parameter table](#hardware-parameters)- can also be used/set in the launch files `bno08x.launch.py` and `bno08x_magnetometer.launch.py` to overrule the defaults set in the related `urdf.xacro`-file.

## Installation

Clone the repository:

```bash
cd ~/ros_ws/src
git clone https://github.com/RbSCR/bno08x_hardware_interface.git
```

Install any missing dependencies:

```bash
cd ~/ros_ws
rosdep install --from-paths src --ignore-src -y
```

Build the package:

```bash
colcon build --packages-select bno08x_hardware_interface
```

## Acknowledgements

This plugin uses the SH-2 protocol library provided by Hillcrest Labs.
It can be found in the `include/sh2` directory.
Visit the official repository here: [SH-2 Protocol Library](https://github.com/ceva-dsp/sh2.git)

This plugin also uses the code from the "BNO08X ROS Driver" package (see link below).
The code has been updated with new functionality that is used in this plugin.
It can be found in the `include/bno08x_driver` and `src/bno08x_driver` directories.

This plugin also uses parts of the code from the "BNO055 Hardware Interface" package
(see link below).

### Inspiration

Inspiration for this package came from:

- the "BNO08X ROS Driver" package by bnbhat (<https://github.com/bnbhat/bno08x_ros2_driver>)
- the "BNO055 Hardware Interface" package by Aditya Kamath
  (<https://github.com/adityakamath/bno055_hardware_interface>)

## License

This package is licensed under the Apache License 2.0. You can find the full license text in the [LICENSE](./LICENSE) file of the repository.
