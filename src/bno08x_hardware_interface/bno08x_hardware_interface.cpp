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


namespace
{

  // Quaternion values for axis remap
struct QVal {
    static constexpr double Zero = 0.0;
    static constexpr double One = 1.0;
    static constexpr double NegOne = -1.0;
    static constexpr double Half = 0.5 ;
    static constexpr double NegHalf = -0.5;
    static constexpr double HalfSqrtTwo = 0.7071067812;
    static constexpr double NegHalfSqrtTwo = -0.7071067812;
};

// Axis remap lookup: East/West North/South Up/Down key -> { Quaternion }
// Names and values from BNO08X datasheet Table 4-3  page 41
const std::map<std::string, sh2_Quaternion_t> kAxisRemap = {
  {"East-North-Up", {QVal::Zero, QVal::Zero, QVal::Zero, QVal::One}},
  {"North-West-Up", {QVal::Zero, QVal::Zero, QVal::HalfSqrtTwo, QVal::HalfSqrtTwo}},
  {"West-South-Up", {QVal::Zero, QVal::Zero, QVal::One, QVal::Zero}},
  {"South-East-Up",  {QVal::Zero, QVal::Zero, QVal::NegHalfSqrtTwo, QVal::HalfSqrtTwo}},
  {"East-South-Down", {QVal::Zero, QVal::NegOne, QVal::Zero, QVal::Zero}},
  {"North-East-Down", {QVal::NegHalfSqrtTwo, QVal::NegHalfSqrtTwo, QVal::Zero, QVal::Zero}},
  {"West-North-Down", {QVal::NegOne, QVal::Zero, QVal::Zero, QVal::Zero}},
  {"South-West-Down", {QVal::NegHalfSqrtTwo, QVal::HalfSqrtTwo, QVal::Zero, QVal::Zero}},
  {"Up-South-East", {QVal::Zero, QVal::NegHalfSqrtTwo, QVal::HalfSqrtTwo, QVal::Zero}},
  {"North-Up-East", {QVal::NegHalf, QVal::NegHalf, QVal::Half, QVal::Half}},
  {"Down-North-East", {QVal::NegHalfSqrtTwo, QVal::Zero, QVal::Zero, QVal::HalfSqrtTwo}},
  {"South-Down-East", {QVal::NegHalf, QVal::Half, QVal::NegHalf, QVal::Half}},
  {"Up-North-West", {QVal::NegHalfSqrtTwo, QVal::Zero, QVal::Zero, QVal::NegHalfSqrtTwo}},
  {"North-Down-West", {QVal::NegHalf, QVal::NegHalf, QVal::NegHalf, QVal::NegHalf}},
  {"Down-South-West", {QVal::Zero, QVal::NegHalfSqrtTwo, QVal::NegHalfSqrtTwo, QVal::Zero}},
  {"South-Up-West", {QVal::Half, QVal::NegHalf, QVal::NegHalf, QVal::Half}},
  {"Up-East-North", {QVal::NegHalf, QVal::NegHalf, QVal::Half, QVal::NegHalf}},
  {"West-Up-North", {QVal::NegHalfSqrtTwo, QVal::Zero, QVal::HalfSqrtTwo, QVal::Zero}},
  {"Down-West-North", {QVal::NegHalf, QVal::Half, QVal::Half, QVal::Half}},
  {"East-Down-North", {QVal::Zero, QVal::NegHalfSqrtTwo, QVal::Zero, QVal::NegHalfSqrtTwo}},
  {"Up-West-South", {QVal::Half, QVal::NegHalf, QVal::Half,  QVal::Half}},
  {"West-Down-South" , {QVal::NegHalfSqrtTwo, QVal::Zero, QVal::NegHalfSqrtTwo, QVal::Zero}},
  {"Down-East-South" , {QVal::NegHalf, QVal::NegHalf, QVal::NegHalf, QVal::Half}},
  {"East-Up-South", {QVal::Zero, QVal::NegHalfSqrtTwo, QVal::Zero, QVal::HalfSqrtTwo}},
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

  // i2c_device
  if (const auto it = info_.hardware_parameters.find("i2c_device");
    it != info_.hardware_parameters.end())
  {
    i2c_device_ = it->second;

    if (i2c_device_.empty() ) {
      RCLCPP_ERROR(logger_, "No i2c_device");
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

  // axis_remap (default: "East-North-Up")
  if (const auto it = info_.hardware_parameters.find("axis_remap");
    it != info_.hardware_parameters.end())
  {
    axis_remap_ = it->second;
  } else {
     RCLCPP_ERROR(logger_, "No axis_remap");
     return hardware_interface::CallbackReturn::ERROR;
  }
  if (kAxisRemap.find(axis_remap_) == kAxisRemap.end()) {
    RCLCPP_ERROR(logger_, "Invalid axis_remap '%s'. "
                          "Must be a valid combination of North | South, East | West, Up | Down, "
                          "with a dash between the 3 words, i.e. format <xxx>-<xxx>-<xxx> "
                          "See BNO08X datasheet page 41.", axis_remap_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // enable_magnetometer (default: false)
  enable_magnetometer_ = parse_bool_param("enable_magnetometer", false);

  // enable_mock_mode (default: false)
  enable_mock_ = parse_bool_param("enable_mock_mode", false);


  // TODO(rbscr)  check sensormode of BNO08X
  // sensor_mode (default: "NDOF")
  if (const auto it = info_.hardware_parameters.find("sensor_mode");
    it != info_.hardware_parameters.end())
  {
    sensor_mode_ = it->second;
  }

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
    "Initialized: i2c_device=%s i2c_addr=0x%02X axis_remap=%s sensor_mode=%s mock=%s",
    i2c_device_.c_str(), i2c_addr_, axis_remap_.c_str(), sensor_mode_.c_str(),
    enable_mock_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}


// ── on_configure: open I2C, init & configure BNO08X ──────────────

hardware_interface::CallbackReturn BNO08XHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring BNO08X...");
  consecutive_read_errors_ = 0;   // TODO(rbscr) check needed for bno08x

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
  try {
    init_communication();
  } catch (const std::exception& e) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Init & configure sensor
  try {
    init_sensor();
  } catch (const std::exception& e) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ENHANCEMENT(rbscr) add chip/sensor info to init_sensor. compare bno0555

  // Remap axis
  sh2_Quaternion_t quat = kAxisRemap.at(axis_remap_);
  if (!this->bno08x_->setReorientation(&quat)) {
    RCLCPP_WARN(logger_, "Failed to remap axis to %s", axis_remap_.c_str());
  } else {
    RCLCPP_INFO(logger_, "Axis remap to %s succesfull", axis_remap_.c_str());
  }

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

void BNO08XHardwareInterface::init_communication()
{
  std::string device = i2c_device_;
    // | std::string address;
    // | this->get_parameter("i2c.bus", device);
    // | this->get_parameter("i2c.address", address);
  RCLCPP_INFO(logger_, "Communication Interface: I2C");
  try
    {
      comm_interface_ = new I2CInterface(device, i2c_addr_);
    }
  catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger_, "Failed to create I2CInterface: %s", e.what());
      throw std::runtime_error("I2CInterface creation failed");
    }
  }


// ENHANCEMENT(rbscr) other functionalities f.e. device info, status, diagnostics
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

  bool imu_report_issues{false};
  if (!this->bno08x_->enable_report(SH2_ROTATION_VECTOR, 1000000 / this->imu_rate_))
  {  // Hz to us
    RCLCPP_ERROR(logger_, "Failed to enable rotation vector sensor");
    imu_report_issues = true;
  }
  if (!this->bno08x_->enable_report(SH2_ACCELEROMETER, 1000000 / this->imu_rate_))
  {  // Hz to us
    RCLCPP_ERROR(logger_, "Failed to enable accelerometer sensor");
    imu_report_issues = true;
  }
  if (!this->bno08x_->enable_report(SH2_GYROSCOPE_CALIBRATED, 1000000 / this->imu_rate_))
  {  // Hz to us
    RCLCPP_ERROR(logger_, "Failed to enable gyroscope sensor");
    imu_report_issues = true;
  }
  if (imu_report_issues)
  {
    RCLCPP_ERROR(logger_, "Failed to enable all 3 IMU reports");
    throw std::runtime_error("BNO08x IMU reports failed");
  }

  if (enable_magnetometer_)
    RCLCPP_INFO(logger_, "Enabling magnetometer");
  {
    if (!this->bno08x_->enable_report(SH2_MAGNETIC_FIELD_CALIBRATED,
          1000000 / this->magnetometer_rate_))
    {  // Hz to us
      RCLCPP_ERROR(logger_, "Failed to enable magnetometer");
      enable_magnetometer_ = false;
    }
  }

  // Initialize the watchdog timer
  auto timeout = std::chrono::milliseconds(2000);  // TODO(rbscr) check watchdog rate
  watchdog_ = new Watchdog();
  watchdog_->set_timeout(timeout);
  watchdog_->set_check_interval(timeout / 2);
  watchdog_->set_callback([this]() {
        RCLCPP_ERROR(logger_, "Watchdog timeout! No data received from sensor. Resetting...");
        this->reset();
  });
  watchdog_->start();
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
  RCLCPP_DEBUG(logger_, "Sensor Callback");
  watchdog_->reset();

  switch (sensor_value->sensorId)
  {
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      if (enable_magnetometer_) {
        // sensor will still return infrequent magnetic field reports even if the report
        // was not enabled, so check it was enabled before publishing.
        hw_magnetic_field_x_ = sensor_value->un.magneticField.x;
        hw_magnetic_field_y_ = sensor_value->un.magneticField.y;
        hw_magnetic_field_z_ = sensor_value->un.magneticField.z;
      }
      break;
    case SH2_ROTATION_VECTOR:
      hw_orientation_x_ = sensor_value->un.rotationVector.i;
      hw_orientation_y_ = sensor_value->un.rotationVector.j;
      hw_orientation_z_ = sensor_value->un.rotationVector.k;
      hw_orientation_w_ = sensor_value->un.rotationVector.real;
      break;
    case SH2_ACCELEROMETER:
      hw_linear_acceleration_x_ = sensor_value->un.accelerometer.x;
      hw_linear_acceleration_y_ = sensor_value->un.accelerometer.y;
      hw_linear_acceleration_z_ = sensor_value->un.accelerometer.z;
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      hw_angular_velocity_x_ = sensor_value->un.gyroscope.x;
      hw_angular_velocity_y_ = sensor_value->un.gyroscope.y;
      hw_angular_velocity_z_ = sensor_value->un.gyroscope.z;
      break;
    default:
      break;
  }
}

void BNO08XHardwareInterface::reset() {
    std::lock_guard<std::mutex> lock(bno08x_mutex_);
    delete bno08x_;
    this->init_sensor();
}

void BNO08XHardwareInterface::close_hardware()
{
  if (!enable_mock_) {
    delete watchdog_;
    delete bno08x_;
    delete comm_interface_;
    RCLCPP_INFO(logger_, "BNO08X hardware closed");
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

  if (enable_magnetometer_) {
    hw_magnetic_field_x_ = 0.0;
    hw_magnetic_field_y_ = 0.0;
    hw_magnetic_field_z_ = 0.0;
  }
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

  int state_count{10};
  if (enable_magnetometer_) {
    state_interfaces.emplace_back(sensor_name, "magnetic_field.x", &hw_magnetic_field_x_);
    state_interfaces.emplace_back(sensor_name, "magnetic_field.y", &hw_magnetic_field_y_);
    state_interfaces.emplace_back(sensor_name, "magnetic_field.z", &hw_magnetic_field_z_);
    state_count += 3;
  }

  RCLCPP_INFO(logger_, "Exported %d state interfaces for sensor '%s'",
                        state_count, sensor_name.c_str());
  return state_interfaces;
}

// ── read: poll BNO08X and update state interfaces ────────────────────────────

hardware_interface::return_type BNO08XHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (enable_mock_) {
    return hardware_interface::return_type::OK;
  }

  // Kind of "dummy" method, needed for ROS2 control hardware-interface.
  // In bno055_hardware_interface read() is used to 'tranfer' the sensorvalues
  // to the state_interface.
  // In this hardware_interface the sensor_callback() 'transfers' the sensorvalues
  // to the state_interface.

  return hardware_interface::return_type::OK;
}

bool BNO08XHardwareInterface::parse_bool_param(
  const std::string & key, bool default_value) const
{
  auto it = info_.hardware_parameters.find(key);
  if (it == info_.hardware_parameters.end()) {return default_value;}
  return it->second == "true";
}

}  // namespace bno08x_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  bno08x_hardware_interface::BNO08XHardwareInterface, hardware_interface::SensorInterface)
