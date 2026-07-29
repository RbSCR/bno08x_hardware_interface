// Copyright 2025 Balachandra Bhat (https://github.com/bnbhat)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <mutex>
#include <chrono>
#include <string>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "bno08x_driver/bno08x.hpp"
#include "bno08x_driver/logger.h"
#include "bno08x_driver/watchdog.hpp"
#include "sh2/sh2.h"



class BNO08xROS : public rclcpp::Node
{
public:
    BNO08xROS();
    ~BNO08xROS();
    void sensor_callback(void *cookie, sh2_SensorValue_t *sensor_value);

private:
    void init_comms();
    void init_parameters();
    void init_sensor();
    void poll_timer_callback();
    void reset();

    // ROS Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
    sensor_msgs::msg::Imu imu_msg_;
    sensor_msgs::msg::MagneticField mag_msg_;
    uint8_t imu_received_flag_;

    // ROS Timer
    rclcpp::TimerBase::SharedPtr poll_timer_;

    // BNO08X Sensor Interface
    BNO08x* bno08x_;
    std::mutex bno08x_mutex_;
    CommInterface* comm_interface_;

    // Watchdog
    Watchdog* watchdog_;

    // Parameters
    std::string frame_id_;
    bool publish_magnetic_field_;
    int magnetic_field_rate_;
    bool publish_imu_;
    int imu_rate_;

    bool publish_orientation_;
    bool publish_acceleration_;
    bool publish_angular_velocity_;
};
