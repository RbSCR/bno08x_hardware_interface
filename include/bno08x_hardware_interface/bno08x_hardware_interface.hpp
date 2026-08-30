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

/**
 * @class BNO08XHardwareInterface
 * @brief ROS2 Control hardware interface for a BNO08X IMU sensor
 *
 *
 * ROS2 Control references
 * - API hardware_interface:
 *   <a href="linkURL">https://docs.ros.org/en/rolling/p/hardware_interface/</a>
 * - Lifecycle:
 *   <a href="linkURL">https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/lifecycle_of_a_hardware_component.html</a>
 *
 *
 */
class BNO08XHardwareInterface : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BNO08XHardwareInterface)

  BNO08XHardwareInterface()
  : logger_(rclcpp::get_logger("BNO08XHardwareInterface")) {}


  /**
   * @brief Parse and check the hardware interface parameters.
   *
   * @param hardware_info
   * @return * hardware_interface::CallbackReturn
   *           SUCCES on succesfull init, ERROR otherwise
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;


  /**
    * @brief Configure the hardware interface and the driver.
    *
    * - Open communication
    * - Init and configure the driver
    *
    * @param previous_state
    * @return * hardware_interface::CallbackReturn
    *           SUCCES on succesfull configure, ERROR otherwise
    */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;


  /**
    * @brief Activate the hardware interface.
    *
    * Logs state change, otherwise standard lifecycle action.
    *
    * @param previous_state
    * @return hardware_interface::CallbackReturn
    */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;


  /**
   * @brief Deactivate the hardware interface.
   *
   * Logs state change, otherwise standard lifecycle action.
   *
   * @param previous_state
   * @return hardware_interface::CallbackReturn
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;


  /**
   * @brief Cleanup resource related to the hardware interface.
   *
   * Closes the hardware.
   *
   * @param previous_state
   * @return hardware_interface::CallbackReturn
   *         SUCCES
   */
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;


  /**
   * @brief Shutdown of the hardware interface.
   *
   * Closes the hardware.
   *
   * @param previous_state
   * @return hardware_interface::CallbackReturn
   *         SUCCES
   */
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;


  /**
   * @brief Handle error situation
   *
   * Called by ros2_control when read() returns ERROR.
   *
   * Closes the hardware.
   *
   * @param previous_state
   * @return hardware_interface::CallbackReturn
   *         SUCCESS (to allow the controller manager a reconfiguration attempt)
   */
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;


  /**
   * @brief Exports the sensor values to the controller
   *
   * Exports the sensor values from the local state storage to the state_interface.
   *
   * Called by the controller.
   *
   * @return std::vector<hardware_interface::StateInterface>
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;


  /**
   * @brief Initiates reading the sensor values
   *
   * Called by the controller.
   *
   * In this hardware interface a kind of "dummy" function, needed for compatability
   * with the ROS2 Control hardware_interface.
   *
   * In the Control hardware_interface framework read() is used to 'tranfer' the sensorvalues
   * to the state_interface.
   * In this hardware_interface the BNO08XHardwareInterface"::"sensor_callback"()""
   * 'transfers' the sensorvalues to a local state storage.
   *
   * @param time
   * @param period
   * @return hardware_interface::return_type
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /**
   * @brief Initialize communication.
   *
   * Currently only I2C is implemented; possible extensions are SPI and UART.
   * Called by BNO08XHardwareInterface"::"on_configure"()"
   *
   * @throws std::runtime_error when communication can not be established.
   *
   */
  void init_communication();

  /**
   * @brief Initialize the sensor.
   *
   * Initializes the sensor and releated timers, enables the required sensor reports.
   *
   * Called by BNO08XHardwareInterface"::"on_configure"()"
   *
   * @throws std::runtime_error when
   * - the BNO08X object can't be allocated
   * - the BNO08X sensor can't be initiated
   * - the IMU sensor reports can't be enabled
   */
  void init_sensor();

  /**
   * @brief Callback function for sensor events.
   *
   * Transfers the appropriate -depending on the actual sensor event- sensor values
   * to a local state storage.
   *
   * @param cookie Pointer to the object that called the function
   * @param sensor_value The sensor value from parsing the sensor event buffer
   */
  void sensor_callback(void* cookie, sh2_SensorValue_t* sensor_value);

  /**
   * @brief Poll the sensor for new events.
   *
   * Called periodically at the rate of the fastest sensor report
   * to get the buffered sensor events.
   *
   * Called by the poll_timer_ timer
   */
  void poll_timer_callback();

  /**
   * @brief Resets the sensor.
   *
   * - deletes the sensor object
   * - and initializes the sensor (again).
   *
   */
  void reset();

  // Suspend the sensor and close the I2C file descriptor.
  // Called by both on_cleanup and on_shutdown.
  /**
   * @brief Closes the hardware.
   *
   * - deletes objects -when mock not enabled- : watchdog, sensoors and communication.
   * - reset local state storage
   *
   * Called by BNO08XHardwareInterface"::"on_cleanup"()"
   * and BNO08XHardwareInterface"::"on_shutdown"()"
   *
   */
  void close_hardware();

  /**
   * @brief Helper funtion to parse a boolean hardware parameter
   *
   * Mirrors the xacro $(arg ...) string convention.
   *
   * @param key
   * @param default_value
   * @return value related to 'key' if key is present, default_value if key is absent
   */
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

  // ROS Timer
  rclcpp::TimerBase::SharedPtr poll_timer_;

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

  const double microtesla_to_tesla_{1e-6};
};

}  // namespace bno08x_hardware_interface

#endif  // BNO08X_HARDWARE_INTERFACE__BNO08X_HARDWARE_INTERFACE_HPP_
