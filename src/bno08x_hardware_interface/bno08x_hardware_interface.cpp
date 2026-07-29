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

/** @file bno08x_hardware_interface.cpp
 *
 * ros2_control SensorInterface for the CEVA BNO08x IMU's over I2C.
 *
 */

#include "bno08x_hardware_interface/bno08x_hardware_interface.hpp"

// TODO(rbscr) check next includes are needed
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"


// TODO(rbscr) temp. file bno055_i2c.h ; check which struct needed to be 'translated'
// |#include "bno055_i2c.h"

namespace
{

// Quaternion raw -> unit: divide by 2^14 (Bosch datasheet §3.6.5.5)
constexpr double QUATERNION_SCALE = 1.0 / 16384.0;   // TODO(rbscr) check needed

// FIXME(rbscr) BNO08X has a different axis remap
// Axis remap lookup: P0-P7 -> { AXIS_MAP_CONFIG byte, AXIS_MAP_SIGN byte }
// Values from BNO055 datasheet Table 3-4 (same as flynneva/bno055)
const std::map<std::string, std::pair<uint8_t, uint8_t>> kAxisRemap = {
  {"P0", {0x21, 0x04}},
  {"P1", {0x24, 0x00}},
  {"P2", {0x24, 0x06}},
  {"P3", {0x21, 0x02}},
  {"P4", {0x24, 0x03}},
  {"P5", {0x21, 0x01}},
  {"P6", {0x21, 0x07}},
  {"P7", {0x24, 0x05}},
};

// TODO(rbscr) check operation mode of BNO08X
// Fusion mode lookup: parameter string -> BNO055 operation mode constant.
// All three modes produce identical outputs: quaternion + angular_velocity + linear_acceleration.
//   NDOF         - 9-DOF, absolute orientation anchored to magnetic North
//   NDOF_FMC_OFF - same as NDOF but fast magnetometer calibration disabled (for noisy environments)
//   IMUPLUS      - 6-DOF, relative orientation, gyro + accel only (no magnetometer)
// |const std::map<std::string, uint8_t> kOperationMode = {
// |  {"NDOF",         BNO055_OPERATION_MODE_NDOF},
// |  {"NDOF_FMC_OFF", BNO055_OPERATION_MODE_NDOF_FMC_OFF},
// |  {"IMUPLUS",      BNO055_OPERATION_MODE_IMUPLUS},
// |};

}  // namespace

namespace bno08x_hardware_interface
{

// ── on_init: parse URDF hardware parameters ──────────────────────────────────

hardware_interface::CallbackReturn BNO08XHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  if (hardware_interface::SensorInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
#pragma GCC diagnostic pop
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Initializing BNO08X hardware interface: %s", info_.name.c_str());

  // TODO(rbscr)  check implemenation onInit [copied from bno055]


  // I2C enable
  enable_i2c_ = parse_bool_param("enable_i2c_comm", true);

  // i2c_bus (default: 1)
  if (const auto it = info_.hardware_parameters.find("i2c_bus");
    it != info_.hardware_parameters.end())
  {
    try {
      i2c_bus_ = std::stoi(it->second);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Invalid i2c_bus: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // i2c_addr (default: 0x4A   alternative: 0x4B)
  if (const auto it = info_.hardware_parameters.find("i2c_addr");
    it != info_.hardware_parameters.end())
  {
    try {
      i2c_addr_ = static_cast<uint8_t>(std::stoul(it->second, nullptr, 16));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Invalid i2c_addr: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // SPI enable
  enable_spi_ = parse_bool_param("enable_spi_comm", false);

  // I2C enable
  enable_uart_ = parse_bool_param("enable_uart_comm", false);

  // Note: SPI and UART communication is not implemented
  // See comm_interface.hhp and i2c / spi / uart_interface.hpp
  // When implemented additional parameter(s) for the specific device will be needed.

  // Check enabled communication(s)
  if (!(enable_i2c_ || enable_spi_ || enable_uart_))
  {
    RCLCPP_ERROR(logger_, "No communication enabled");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!exactly_one_boolean_true(enable_i2c_, enable_spi_, enable_uart_))
  {
    RCLCPP_ERROR(logger_, "Multiple communications enabled. Enable only one.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // axis_remap (default: "P1")
  if (const auto it = info_.hardware_parameters.find("axis_remap");
    it != info_.hardware_parameters.end())
  {
    axis_remap_ = it->second;
  }
  if (kAxisRemap.find(axis_remap_) == kAxisRemap.end()) {
    RCLCPP_ERROR(logger_, "Invalid axis_remap '%s'. Must be P0-P7.", axis_remap_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // enable_mock_mode (default: false)
  enable_mock_ = parse_bool_param("enable_mock_mode", false);


  // TODO(rbscr)  check sensormode of BNO08X
  // sensor_mode (default: "NDOF")
  if (const auto it = info_.hardware_parameters.find("sensor_mode");
    it != info_.hardware_parameters.end())
  {
    sensor_mode_ = it->second;
  }

  // CLEANUP(rbscr) OperationMode
  // |if (kOperationMode.find(sensor_mode_) == kOperationMode.end()) {
  // |  RCLCPP_ERROR(
  // |    logger_, "Invalid sensor_mode '%s'. Must be one of: NDOF, NDOF_FMC_OFF, IMUPLUS.",
  // |    sensor_mode_.c_str());
  // |  return hardware_interface::CallbackReturn::ERROR;
  // |}

  if (info_.sensors.size() != 1) {
    RCLCPP_ERROR(logger_, "Expected exactly 1 <sensor> element, got %zu", info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate that each declared state interface name matches one of the 10 expected.
  const std::vector<std::string> kExpected = {
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",
  };
  for (const auto & si : info_.sensors[0].state_interfaces) {
    if (std::find(kExpected.begin(), kExpected.end(), si.name) == kExpected.end()) {
      RCLCPP_ERROR(
        logger_, "Unexpected state interface '%s'. Expected one of: "
        "orientation.{x,y,z,w}, angular_velocity.{x,y,z}, linear_acceleration.{x,y,z}",
        si.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    logger_,
    "Initialized: i2c_bus=%d i2c_addr=0x%02X axis_remap=%s sensor_mode=%s mock=%s",
    i2c_bus_, i2c_addr_, axis_remap_.c_str(), sensor_mode_.c_str(),
    enable_mock_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}



// ── on_configure: open I2C, init & configure BNO08X ──────────────

// TODO(rbscr) check lifecycle method on_configure

hardware_interface::CallbackReturn BNO08XHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring BNO08X...");
  consecutive_read_errors_ = 0;   //  TODO(rbscr) check relaly needed for bno08x

  if (enable_mock_) {
    RCLCPP_INFO(logger_, "Mock mode enabled - skipping communication initialization");
    // Identity quaternion: represents no rotation (robot aligned with world frame).
    // A zero quaternion is mathematically invalid and would produce NaN in the EKF.
    hw_orientation_w_ = 1.0;
    hw_orientation_x_ = 0.0;
    hw_orientation_y_ = 0.0;
    hw_orientation_z_ = 0.0;
    // Angular velocity and linear acceleration stay at 0.0 (correct for a stationary mock).
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Open communication (I2C bus)
  std::string device = "/dev/i2c-" + std::to_string(i2c_bus_);
  try {
    init_communication();
  } catch (const std::exception& e) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // | if (bno055_i2c_open(device.c_str(), i2c_addr_) != 0) {
  // |   RCLCPP_ERROR(logger_, "Failed to open I2C device %s at address 0x%02X",
  // |     device.c_str(), i2c_addr_);
  // |   return hardware_interface::CallbackReturn::ERROR;
  // | }

  // Init & configure sensor
  try {
    init_sensor();
  } catch (const std::exception& e) {
    // TODO(rbscr) close comm ? compare bno055 (see below)
    return hardware_interface::CallbackReturn::ERROR;
  }

  // | // Wire Bosch driver callbacks
  // | sensor_.bus_read  = BNO055_I2C_bus_read;
  // | sensor_.bus_write = BNO055_I2C_bus_write;
  // | sensor_.delay_msec = BNO055_delay_msek;
  // | sensor_.dev_addr  = static_cast<u8>(i2c_addr_);

  // | // Initialize the Bosch driver (reads chip ID, populates rev info)
  // | s32 comres = bno055_init(&sensor_);
  // | if (comres != BNO055_SUCCESS) {
  // | RCLCPP_ERROR(logger_,"bno055_init() failed (err=%d). Check wiring and I2C address.", comres);
  // |   bno055_i2c_close();
  // |   return hardware_interface::CallbackReturn::ERROR;
  // | }

  // TODO(rbscr) add chip/sensor info to init_sensor. compare bno0555 (see below)
  // | RCLCPP_INFO(
  // |   logger_, "BNO08X detected: chip_id=0x%02X sw_rev=0x%04X",
  // |   sensor_.chip_id, sensor_.sw_rev_id);

  // TODO(rbscr) check config init_sensor. compare bno0555 (see below)
  // | // Switch to CONFIG mode to apply settings
  // | comres = bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
  // | if (comres != BNO055_SUCCESS) {
  // |   RCLCPP_WARN(logger_, "Failed to set CONFIG mode (err=%d)", comres);
  // | }
  // | rclcpp::sleep_for(std::chrono::milliseconds(25));  // datasheet: >=19 ms

  // | // Normal power mode
  // | comres = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
  // | if (comres != BNO055_SUCCESS) {
  // |   RCLCPP_WARN(logger_, "Failed to set NORMAL power mode (err=%d)", comres);
  // | }

  // | // Gyro unit: radians per second
  // | comres = bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);
  // | if (comres != BNO055_SUCCESS) {
  // |   RCLCPP_ERROR(logger_, "Failed to set gyro unit to rps (err=%d)", comres);
  // |   bno055_i2c_close();
  // |   return hardware_interface::CallbackReturn::ERROR;
  // | }

  // | // Accel unit: m/s^2
  // | comres = bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
  // | if (comres != BNO055_SUCCESS) {
  // |   RCLCPP_ERROR(logger_, "Failed to set accel unit to m/s^2 (err=%d)", comres);
  // |   bno055_i2c_close();
  // |   return hardware_interface::CallbackReturn::ERROR;
  // | }

  // FIXME(rbscr) Use axis remap in init_sensor ? compare bno055 (see below)
  // | // Axis remap: write { AXIS_MAP_CONFIG, AXIS_MAP_SIGN } directly via the
  // | // wired bus_write callback (same values as flynneva/bno055 P-code table)
  // | const auto & remap = kAxisRemap.at(axis_remap_);
  // | u8 remap_cfg  = remap.first;
  // | u8 remap_sign = remap.second;
  // | if (sensor_.bus_write(sensor_.dev_addr, BNO055_AXIS_MAP_CONFIG_ADDR, &remap_cfg, 1) != 0) {
  // |   RCLCPP_WARN(logger_, "Axis remap: failed to write AXIS_MAP_CONFIG register");
  // | }
  // | if (sensor_.bus_write(sensor_.dev_addr, BNO055_AXIS_MAP_SIGN_ADDR, &remap_sign, 1) != 0) {
  // |   RCLCPP_WARN(logger_, "Axis remap: failed to write AXIS_MAP_SIGN register");
  // | }
  // | RCLCPP_INFO(logger_, "Axis remap %s applied (cfg=0x%02X sign=0x%02X)",
  // |   axis_remap_.c_str(), remap_cfg, remap_sign);

  // | // Switch to configured fusion mode
  // | comres = bno055_set_operation_mode(kOperationMode.at(sensor_mode_));
  // | if (comres != BNO055_SUCCESS) {
  // |   RCLCPP_ERROR(
  // |     logger_, "Failed to set %s operation mode (err=%d)", sensor_mode_.c_str(), comres);
  // |   bno055_i2c_close();
  // |   return hardware_interface::CallbackReturn::ERROR;
  // | }
  // | rclcpp::sleep_for(std::chrono::milliseconds(20));  // datasheet: >=7 ms

  // | RCLCPP_INFO(logger_, "BNO055 configured in %s fusion mode", sensor_mode_.c_str());
  // | return hardware_interface::CallbackReturn::SUCCESS;

  RCLCPP_INFO(logger_, "BNO08X initialized and configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}


// ── on_activate / on_deactivate ─────────────────────────────────

hardware_interface::CallbackReturn BNO08XHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "BNO08X hardware interface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO08XHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "BNO08X hardware interface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_cleanup / on_shutdown ─────────────────────────────────

hardware_interface::CallbackReturn BNO08XHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_hardware();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO08XHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_hardware();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_error ─────────────────────────────────

hardware_interface::CallbackReturn BNO08XHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(logger_, "BNO08X hardware interface entering error recovery — closing hardware");
  close_hardware();
  return hardware_interface::CallbackReturn::SUCCESS;
}


// Initialize communications

// FIXME  copied (and adapted) from void BNO08xROS::init_comms()
void BNO08XHardwareInterface::init_communication()
{
  // | bool i2c_enabled, uart_enabled, spi_enabled;
  // | this->get_parameter("i2c.enabled", i2c_enabled);
  // | this->get_parameter("uart.enabled", uart_enabled);
  // | this->get_parameter("spi.enabled", spi_enabled);

  if (enable_i2c_) {
    std::string device = "/dev/i2c-" + std::to_string(i2c_bus_);
    // | std::string address;
    // | this->get_parameter("i2c.bus", device);
    // | this->get_parameter("i2c.address", address);
    RCLCPP_INFO(logger_, "Communication Interface: I2C");
    try
    {
      // | comm_interface_ = new I2CInterface(device, std::stoi(address, nullptr, 16));
      // TODO check 3e param
      comm_interface_ = new I2CInterface(device, i2c_addr_);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger_, "Failed to create I2CInterface: %s", e.what());
      throw std::runtime_error("I2CInterface creation failed");
    }
  } else {
    if (enable_uart_) {
      RCLCPP_INFO(logger_, "Communication Interface: UART");
      std::string device = "dummy-uart-device";
      // | this->get_parameter("uart.device", device);
      try
      {
        comm_interface_ = new UARTInterface(device);
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(logger_, "UART Interface not implemented: %s", e.what());
        throw std::runtime_error("UARTInterface creation failed");
      }
    } else {
      if (enable_spi_) {
        RCLCPP_INFO(logger_, "Communication Interface: SPI");
        std::string device = "dummy-spi-device";
        // | this->get_parameter("spi.device", device);
        try
        {
          comm_interface_ = new SPIInterface(device);
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(logger_, "SPI Interface not implemented: %s", e.what());
          throw std::runtime_error("SPIInterface creation failed");
        }
      } else {
        RCLCPP_ERROR(logger_, "No communication interface enabled!");
        throw std::runtime_error("Communication interface setup failed");
      }
    }
  }
}


// FIXME  copied (and adapted) from void BNO08xROS::init_sensor()
// TODO(rbscr) update/fix init_sensor()
/**
 * @brief Initialize the sensor
 *
 * This function initializes the sensor and enables the required sensor reports
 *
 */
void BNO08XHardwareInterface::init_sensor()
{
  try {
    bno08x_ = new BNO08x(comm_interface_, std::bind(&BNO08XHardwareInterface::sensor_callback,
      this, std::placeholders::_1, std::placeholders::_2), this);
  }
  catch (const std::bad_alloc& e)
  {
    RCLCPP_ERROR(logger_, "Failed to allocate memory for BNO08x object: %s", e.what());
    throw std::runtime_error("BNO08x object allocation failed");
  }

  if (!bno08x_->begin())
  {
    RCLCPP_ERROR(logger_, "Failed to initialize BNO08X sensor");
    throw std::runtime_error("BNO08x initialization failed");
  }

  if (bn8ros_publish_magnetic_field_)
  {
    if (!this->bno08x_->enable_report(SH2_MAGNETIC_FIELD_CALIBRATED,
          1000000 / this->magnetic_field_rate_))
    {  // Hz to us
      RCLCPP_ERROR(logger_, "Failed to enable magnetic field sensor");
    }
  }

  if (bn8ros_publish_imu_)
  {
    if (!this->bno08x_->enable_report(SH2_ROTATION_VECTOR, 1000000 / this->imu_rate_))
    {  // Hz to us
      RCLCPP_ERROR(logger_, "Failed to enable rotation vector sensor");
    }
    if (!this->bno08x_->enable_report(SH2_ACCELEROMETER, 1000000 / this->imu_rate_))
    {  // Hz to us
      RCLCPP_ERROR(logger_, "Failed to enable accelerometer sensor");
    }
    if (!this->bno08x_->enable_report(SH2_GYROSCOPE_CALIBRATED, 1000000 / this->imu_rate_))
    {  // Hz to us
      RCLCPP_ERROR(logger_, "Failed to enable gyroscope sensor");
    }
  }
  if (!(bn8ros_publish_imu_ || bn8ros_publish_magnetic_field_))
  {
    RCLCPP_ERROR(logger_, "No sensor reports enabled! Exiting...");
    throw std::runtime_error("No sensor reports enabled");
  }
}


/**
 * @brief Callback function for sensor events
 *
 * @param cookie Pointer to the object that called the function, not used here
 * @param sensor_value The sensor value from parsing the sensor event buffer
 *
 */
void BNO08XHardwareInterface::sensor_callback(void* cookie, sh2_SensorValue_t* sensor_value)
{
  DEBUG_LOG("Sensor Callback");
  watchdog_->reset();               // TODO(rbscr) check watchdog from bno08x_ros needed
                                    // if yes: add watchdog  intialize and reset
                                    // also check needed rate (bno055 hardware interface)
  switch (sensor_value->sensorId)
  {
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      // FIXME(rbscr) hardware interface does not publish messages
      // instead it transfers the values to allow ROS2 Control to publish
      hw_magnetic_field_x_ = sensor_value->un.magneticField.x;
      hw_magnetic_field_y_ = sensor_value->un.magneticField.y;
      hw_magnetic_field_z_ = sensor_value->un.magneticField.z;

      // | this->mag_msg_.magnetic_field.x = sensor_value->un.magneticField.x;
      // | this->mag_msg_.magnetic_field.y = sensor_value->un.magneticField.y;
      // | this->mag_msg_.magnetic_field.z = sensor_value->un.magneticField.z;
      // | this->mag_msg_.header.frame_id = this->frame_id_;
      // | this->mag_msg_.header.stamp = this->get_clock()->now();
      // | // IMU will still return infrequent magnetic field reports even if the report
      // | // was not enabled, so check it was enabled before publishing.
      // | if (publish_magnetic_field_) {
      // |   this->mag_publisher_->publish(this->mag_msg_);
      // | }
      break;
    case SH2_ROTATION_VECTOR:
      // FIXME(rbscr) hadware interface does not publish messages (see above)
      hw_orientation_x_ = sensor_value->un.rotationVector.i;
      hw_orientation_y_ = sensor_value->un.rotationVector.j;
      hw_orientation_z_ = sensor_value->un.rotationVector.k;
      hw_orientation_w_ = sensor_value->un.rotationVector.real;

      // | this->imu_msg_.orientation.x = sensor_value->un.rotationVector.i;
      // | this->imu_msg_.orientation.y = sensor_value->un.rotationVector.j;
      // | this->imu_msg_.orientation.z = sensor_value->un.rotationVector.k;
      // | this->imu_msg_.orientation.w = sensor_value->un.rotationVector.real;
      // | imu_received_flag_ |= ROTATION_VECTOR_RECEIVED;
      break;
    case SH2_ACCELEROMETER:
      // FIXME(rbscr) hardware interface does not publish messages (see above)
      hw_linear_acceleration_x_ = sensor_value->un.accelerometer.x;
      hw_linear_acceleration_y_ = sensor_value->un.accelerometer.y;
      hw_linear_acceleration_z_ = sensor_value->un.accelerometer.z;

      // | this->imu_msg_.linear_acceleration.x = sensor_value->un.accelerometer.x;
      // | this->imu_msg_.linear_acceleration.y = sensor_value->un.accelerometer.y;
      // | this->imu_msg_.linear_acceleration.z = sensor_value->un.accelerometer.z;
      // | imu_received_flag_ |= ACCELEROMETER_RECEIVED;
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      // FIXME(rbscr) hardware interface does not publish messages (see above)
      hw_angular_velocity_x_ = sensor_value->un.gyroscope.x;
      hw_angular_velocity_y_ = sensor_value->un.gyroscope.y;
      hw_angular_velocity_z_ = sensor_value->un.gyroscope.z;

      // | this->imu_msg_.angular_velocity.x = sensor_value->un.gyroscope.x;
      // | this->imu_msg_.angular_velocity.y = sensor_value->un.gyroscope.y;
      // | this->imu_msg_.angular_velocity.z = sensor_value->un.gyroscope.z;
      // | imu_received_flag_ |= GYROSCOPE_RECEIVED;
      break;
    default:
      break;
  }

  // TODO(rbscr) check how bn055_hardware_interface iniates the transfer

  // FIXME(rbscr) hardware interface does not publish messages (see above)
  // | if (imu_received_flag_ == (ROTATION_VECTOR_RECEIVED | ACCELEROMETER_RECEIVED
  // |                            | GYROSCOPE_RECEIVED))
  // | {
  // |   this->imu_msg_.header.frame_id = this->frame_id_;
  // |   this->imu_msg_.header.stamp = this->get_clock()->now();
  // |   this->imu_publisher_->publish(this->imu_msg_);
  // |   imu_received_flag_ = 0;
  // | }
}

/**
 * @brief Poll the sensor for new events
 *
 * This function is called periodically at the rate of the fastest sensor report
 * to get the buffered sensor events
 * called by the poll_timer_ timer
 */
 // TODO(rbscr) fix/update
void BNO08XHardwareInterface::poll_timer_callback()
{
  {
    std::lock_guard<std::mutex> lock(bno08x_mutex_);
    this->bno08x_->poll();
  }
}

void BNO08XHardwareInterface::close_hardware()
{
  if (!enable_mock_) {
// |    bno055_set_power_mode(BNO055_POWER_MODE_SUSPEND);
// TODO(rbscr) close communication
// |    bno055_i2c_close();
    RCLCPP_INFO(logger_, "BNO08X suspended and I2C closed");
  }
  hw_orientation_x_ = 0.0;
  hw_orientation_y_ = 0.0;
  hw_orientation_z_ = 0.0;
  hw_orientation_w_ = 1.0;
  hw_angular_velocity_x_    = 0.0;
  hw_angular_velocity_y_    = 0.0;
  hw_angular_velocity_z_    = 0.0;
  hw_linear_acceleration_x_ = 0.0;
  hw_linear_acceleration_y_ = 0.0;
  hw_linear_acceleration_z_ = 0.0;

  hw_magnetic_field_x_ = 0.0;  // TODO(rbscr) magnetic field currently not used
  hw_magnetic_field_y_ = 0.0;
  hw_magnetic_field_z_ = 0.0;
}

// ── export_state_interfaces ───────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
BNO08XHardwareInterface::export_state_interfaces()
{
  const std::string & sensor_name = info_.sensors[0].name;
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(sensor_name, "orientation.x",         &hw_orientation_x_);
  state_interfaces.emplace_back(sensor_name, "orientation.y",         &hw_orientation_y_);
  state_interfaces.emplace_back(sensor_name, "orientation.z",         &hw_orientation_z_);
  state_interfaces.emplace_back(sensor_name, "orientation.w",         &hw_orientation_w_);
  state_interfaces.emplace_back(sensor_name, "angular_velocity.x",    &hw_angular_velocity_x_);
  state_interfaces.emplace_back(sensor_name, "angular_velocity.y",    &hw_angular_velocity_y_);
  state_interfaces.emplace_back(sensor_name, "angular_velocity.z",    &hw_angular_velocity_z_);
  state_interfaces.emplace_back(sensor_name, "linear_acceleration.x", &hw_linear_acceleration_x_);
  state_interfaces.emplace_back(sensor_name, "linear_acceleration.y", &hw_linear_acceleration_y_);
  state_interfaces.emplace_back(sensor_name, "linear_acceleration.z", &hw_linear_acceleration_z_);

  // TODO(rbscr)  when magnetic field will be used add x / y / z values here

  RCLCPP_INFO(logger_, "Exported 10 state interfaces for sensor '%s'", sensor_name.c_str());
  return state_interfaces;
}

// FIXME(rbscr) read() needed for compliance with hardware_component_interface
// sensor_callback() [bno08x] puts the imu values in the state_interface

// ── read: poll BNO055 and update state interfaces ────────────────────────────

hardware_interface::return_type BNO08XHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (enable_mock_) {
    return hardware_interface::return_type::OK;
  }
// |
// |   // Orientation — raw quaternion (s16 values, scale = 1/2^14)
// |   bno055_quaternion_t quat;
// |   // Angular velocity — raw s16 counts; unit was set to RPS in on_configure
// |   bno055_gyro_t gyro;
// |   // Linear acceleration — raw s16 counts; unit was set to m/s² in on_configure
// |   bno055_linear_accel_t lin_accel;
// |
// |   // Use raw read functions + manual scaling to avoid the convert_double variants,
// |   // which re-read the unit register on every call and may trigger a CONFIG mode
// |   // switch if the register is ever corrupted (stalling fusion for 19+ ms).
// |   bool ok =
// |     (bno055_read_quaternion_wxyz(&quat) == BNO055_SUCCESS) &&
// |     (bno055_read_gyro_xyz(&gyro) == BNO055_SUCCESS) &&
// |     (bno055_read_linear_accel_xyz(&lin_accel) == BNO055_SUCCESS);
// |
// |   if (!ok) {
// |     ++consecutive_read_errors_;
// |     if (consecutive_read_errors_ >= 10) {
// |       RCLCPP_ERROR(
// |         logger_, "BNO08X: %d consecutive read failures — reporting ERROR",
// |         consecutive_read_errors_);
// |       return hardware_interface::return_type::ERROR;
// |     }
// |     RCLCPP_WARN(logger_, "BNO08X read error (streak=%d) — keeping previous values",
// |       consecutive_read_errors_);
// |     return hardware_interface::return_type::OK;
// |   }
// |   consecutive_read_errors_ = 0;
// |
// |   // Normalize quaternion (raw values scale to unit quaternion via QUATERNION_SCALE)
// |   const double qw = quat.w * QUATERNION_SCALE;
// |   const double qx = quat.x * QUATERNION_SCALE;
// |   const double qy = quat.y * QUATERNION_SCALE;
// |   const double qz = quat.z * QUATERNION_SCALE;
// |   const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
// |   if (norm > 1e-6) {
// |     hw_orientation_w_ = qw / norm;
// |     hw_orientation_x_ = qx / norm;
// |     hw_orientation_y_ = qy / norm;
// |     hw_orientation_z_ = qz / norm;
// |   }
// |
// |   hw_angular_velocity_x_ = gyro.x / BNO055_GYRO_DIV_RPS;
// |   hw_angular_velocity_y_ = gyro.y / BNO055_GYRO_DIV_RPS;
// |   hw_angular_velocity_z_ = gyro.z / BNO055_GYRO_DIV_RPS;
// |
// |   hw_linear_acceleration_x_ = lin_accel.x / BNO055_LINEAR_ACCEL_DIV_MSQ;
// |   hw_linear_acceleration_y_ = lin_accel.y / BNO055_LINEAR_ACCEL_DIV_MSQ;
// |   hw_linear_acceleration_z_ = lin_accel.z / BNO055_LINEAR_ACCEL_DIV_MSQ;

  return hardware_interface::return_type::OK;
}

bool BNO08XHardwareInterface::parse_bool_param(
  const std::string & key, bool default_value) const
{
  auto it = info_.hardware_parameters.find(key);
  if (it == info_.hardware_parameters.end()) {return default_value;}
  return it->second == "true";
}

bool BNO08XHardwareInterface::exactly_one_boolean_true(bool a, bool b, bool c) {
    return (a && !b && !c) || (!a && b && !c) || (!a && !b && c);
}
}  // namespace bno08x_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  bno08x_hardware_interface::BNO08XHardwareInterface, hardware_interface::SensorInterface)
