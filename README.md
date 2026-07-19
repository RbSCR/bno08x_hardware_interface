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

## Features
