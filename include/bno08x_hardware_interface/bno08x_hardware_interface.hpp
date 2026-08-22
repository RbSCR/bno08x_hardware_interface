// Copyright 2026 RbSCR
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

#ifndef BNO08X_HARDWARE_INTERFACE__BNO08X_HARDWARE_INTERFACE_HPP_
#define BNO08X_HARDWARE_INTERFACE__BNO08X_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"  // link: https://github.com/ros-controls/ros2_control/blob/kilted/hardware_interface/include/hardware_interface/sensor_interface.hpp
#include "hardware_interface/handle.hpp"  // link: https://github.com/ros-controls/ros2_control/blob/kilted/hardware_interface/include/hardware_interface/handle.hpp
#include "hardware_interface/hardware_info.hpp"  // link: https://github.com/ros-controls/ros2_control/blob/kilted/hardware_interface/include/hardware_interface/hardware_info.hpp
#include "hardware_interface/types/hardware_interface_return_values.hpp"  // link: https://github.com/ros-controls/ros2_control/blob/kilted/hardware_interface/include/hardware_interface/types/hardware_interface_return_values.hpp

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "bno08x_driver/bno08x.hpp"
#include "bno08x_driver/i2c_interface.hpp"
#include "bno08x_driver/uart_interface.hpp"
#include "bno08x_driver/spi_interface.hpp"
#include "bno08x_driver/watchdog.hpp"

namespace bno08x_hardware_interface
{

class BNO08XHardwareInterface : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BNO08XHardwareInterface)

  BNO08XHardwareInterface()
  : logger_(rclcpp::get_logger("BNO08XHardwareInterface")) {}

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  // Called by ros2_control when read() returns ERROR (after 10 consecutive failures).
  // The base-class default returns ERROR → FINALIZED, bypassing on_cleanup and leaking
  // the I2C fd. Override to close hardware and return SUCCESS → UNCONFIGURED so the
  // controller manager can attempt reconfiguration without a process restart.
  // TODO(rbscr) check implementation in bn055 hardware interface ; check comment
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // read() needed for compliance with hardware_component_interface
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Initialize communication with the sensor.
  // Currently only I2C is implemented; possible extensions are SPI and UART.
  // Called by on_configure()
  // Throws std::runtime_error when communication can not be established.
  void init_communication();

  // Initialize the sensor
  // Called by on_configure()
  void init_sensor();

  // Callback function for sensor events
  void sensor_callback(void* cookie, sh2_SensorValue_t* sensor_value);

  // Poll the sensor for new events
  void poll_timer_callback();

  void reset();

  // Suspend the sensor and close the I2C file descriptor.
  // Called by both on_cleanup and on_shutdown.
  void close_hardware();

  // Parse a boolean hardware parameter; returns default_value if the key is absent.
  // Accepts only "true" — mirrors the xacro $(arg ...) string convention.
  bool parse_bool_param(const std::string & key, bool default_value) const;

  //
  rclcpp::Logger logger_;

  // Parameters
  int         i2c_bus_{1};
  uint8_t     i2c_addr_{0x4A};  // Default 0x4A, alternative 0x4B  Par 1.2.2.1 Datasheet BNO08X
  std::string axis_remap_{"East-North-Up"};

  bool enable_magnetometer_{false};
  int magnetometer_rate_{100};  // report frequency in Hz.
  int imu_rate_{100};           // report frequency in Hz.

  bool enable_mock_{false};

  // Consecutive read failures before returning ERROR (threshold = 10)
  int consecutive_read_errors_{0};  // TODO(rbscr) check used / needed in bno08x

  // BNO08X Sensor Interface
  BNO08x* bno08x_;
  std::mutex bno08x_mutex_;
  CommInterface* comm_interface_;

  // Watchdog
  Watchdog* watchdog_;

  // State storage for imu_sensor - always -- 10 interfaces
  double hw_orientation_x_{0.0};
  double hw_orientation_y_{0.0};
  double hw_orientation_z_{0.0};
  double hw_orientation_w_{1.0};
  double hw_angular_velocity_x_{0.0};
  double hw_angular_velocity_y_{0.0};
  double hw_angular_velocity_z_{0.0};
  double hw_linear_acceleration_x_{0.0};
  double hw_linear_acceleration_y_{0.0};
  double hw_linear_acceleration_z_{0.0};
  // State storage for magnetometer - optional -- 3 interfaces
  double hw_magnetic_field_x_{0.0};
  double hw_magnetic_field_y_{0.0};
  double hw_magnetic_field_z_{0.0};
};

}  // namespace bno08x_hardware_interface

#endif  // BNO08X_HARDWARE_INTERFACE__BNO08X_HARDWARE_INTERFACE_HPP_
