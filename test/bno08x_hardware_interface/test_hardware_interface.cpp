// Copyright 2026 RbSCR
// Copyright 2025 Aditya Kamath (https://github.com/adityakamath)

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "bno08x_hardware_interface/bno08x_hardware_interface.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

// ── helpers ───────────────────────────────────────────────────────────────────

hardware_interface::HardwareInfo make_valid_imu_info(
  const std::string & i2c_device = "/dev/i2c-bn08x",
  const std::string & i2c_addr = "4A",
  const std::string & axis_remap = "East-North-Up")
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["i2c_device"] = i2c_device;
  info.hardware_parameters["i2c_addr"]   = i2c_addr;
  info.hardware_parameters["axis_remap"] = axis_remap;

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bno08x";
  for (const auto & name : {
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"})
  {
    hardware_interface::InterfaceInfo iface;
    iface.name = name;
    sensor.state_interfaces.push_back(iface);
  }
  info.sensors.push_back(sensor);
  return info;
}

static rclcpp_lifecycle::State unconfigured_state()
{
  return rclcpp_lifecycle::State(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
}
static rclcpp_lifecycle::State inactive_state()
{
  return rclcpp_lifecycle::State(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
}
static rclcpp_lifecycle::State active_state()
{
  return rclcpp_lifecycle::State(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
}

// ── on_init ───────────────────────────────────────────────────────────────────

TEST(InitTest, ValidParams)
{
  auto init = [](hardware_interface::HardwareInfo info) {
      bno08x_hardware_interface::BNO08XHardwareInterface hw;
      return hw.on_init(info);
    };

  EXPECT_EQ(init(make_valid_imu_info()), CallbackReturn::SUCCESS);
  EXPECT_EQ(init(make_valid_imu_info("/dev/i2c-bn08x", "4B")), CallbackReturn::SUCCESS);
  // device /dev/i2c-bn08x, addr 0x4B

  auto magnetometer_enabled = make_valid_imu_info();
  magnetometer_enabled.hardware_parameters["enable_magnetometer"] = "true";
  EXPECT_EQ(init(magnetometer_enabled), CallbackReturn::SUCCESS);

  auto magnetometer_disabled = make_valid_imu_info();
  magnetometer_disabled.hardware_parameters["enable_magnetometer"] = "false";
  EXPECT_EQ(init(magnetometer_disabled), CallbackReturn::SUCCESS);

  auto mock_enabled = make_valid_imu_info();
  mock_enabled.hardware_parameters["enable_mock_mode"] = "true";
  EXPECT_EQ(init(mock_enabled), CallbackReturn::SUCCESS);

  auto calib = make_valid_imu_info();
  calib.hardware_parameters["calib_file"] = "~/.ros/bno08x_calib.yaml";
  EXPECT_EQ(init(calib), CallbackReturn::SUCCESS);
}

TEST(InitTest, ValidAllSensorModes)
{
  for (const auto & mode : {"NDOF", "NDOF_FMC_OFF", "IMUPLUS"}) {
    bno08x_hardware_interface::BNO08XHardwareInterface hw;
    auto info = make_valid_imu_info();
    info.hardware_parameters["sensor_mode"] = mode;
    EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS) << "sensor_mode=" << mode;
  }
}

TEST(InitTest, ValidAllAxisRemaps)
{
  for (const auto & remap : {
          "East-North-Up", "North-West-Up", "West-South-Up", "South-East-Up",
          "East-South-Down", "North-East-Down", "West-North-Down", "South-West-Down",
          "Up-South-East", "North-Up-East", "Down-North-East", "South-Down-East",
          "Up-North-West", "North-Down-West", "Down-South-West", "South-Up-West",
          "Up-East-North", "West-Up-North", "Down-West-North", "East-Down-North",
          "Up-West-South", "West-Down-South", "Down-East-South", "East-Up-South"}) {
    bno08x_hardware_interface::BNO08XHardwareInterface hw;
    EXPECT_EQ(hw.on_init(make_valid_imu_info("/dev/i2c-bno08x", "4A", remap)),
              CallbackReturn::SUCCESS) << "axis_remap=" << remap;
  }
}

TEST(InitTest, InvalidParamsFail)
{
  auto init = [](hardware_interface::HardwareInfo info) {
      bno08x_hardware_interface::BNO08XHardwareInterface hw;
      return hw.on_init(info);
    };

  auto no_sensor = make_valid_imu_info();
  no_sensor.sensors.clear();
  EXPECT_EQ(init(no_sensor), CallbackReturn::ERROR);

  auto two_sensors = make_valid_imu_info();
  two_sensors.sensors.push_back(two_sensors.sensors[0]);
  EXPECT_EQ(init(two_sensors), CallbackReturn::ERROR);

  // wrong axis_remap
  EXPECT_EQ(init(make_valid_imu_info("/dev/i2c-bno08x", "4A", "East-East-North")),
            CallbackReturn::ERROR);

   EXPECT_EQ(init(make_valid_imu_info("/dev/i2c-bno08x", "4A", "Up-Down-South")),
            CallbackReturn::ERROR);

  // empty i2c_device
  EXPECT_EQ(init(make_valid_imu_info("", "4A", "East-North-Up" )), CallbackReturn::ERROR);

  auto empty_remap = make_valid_imu_info();
  empty_remap.hardware_parameters["axis_remap"] = "";
  EXPECT_EQ(init(empty_remap), CallbackReturn::ERROR);

  // TODO(rbscr) Bad device test temporarily disabled
  // auto bad_device = make_valid_imu_info();
  // bad_device.hardware_parameters["i2c_device"] = "abc";
  // EXPECT_EQ(init(bad_device), CallbackReturn::ERROR);

  auto bad_addr = make_valid_imu_info();
  bad_addr.hardware_parameters["i2c_addr"] = "XZ";
  EXPECT_EQ(init(bad_addr), CallbackReturn::ERROR);

  // TODO(rbscr) Bad mode test temporareily disabled
  // auto bad_mode = make_valid_imu_info();
  // bad_mode.hardware_parameters["sensor_mode"] = "ACCGYRO";
  // EXPECT_EQ(init(bad_mode), CallbackReturn::ERROR);

  // Unknown state interface name must be rejected
  auto bad_iface = make_valid_imu_info();
  bad_iface.sensors[0].state_interfaces.push_back([] {
      hardware_interface::InterfaceInfo i;
      i.name = "orientation.q";
      return i;
    }());
  EXPECT_EQ(init(bad_iface), CallbackReturn::ERROR);
}

// ── export_state_interfaces ───────────────────────────────────────────────────

TEST(ExportStateInterfacesTest, InterfacesCorrect)
{
  bno08x_hardware_interface::BNO08XHardwareInterface hw;
  ASSERT_EQ(hw.on_init(make_valid_imu_info()), CallbackReturn::SUCCESS);
  auto ifaces = hw.export_state_interfaces();

  ASSERT_EQ(ifaces.size(), 10u);

  std::vector<std::string> names;
  for (const auto & iface : ifaces) {
    EXPECT_EQ(iface.get_prefix_name(), "bno08x");
    names.push_back(iface.get_interface_name());
  }
  for (const auto & expected : {
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"})
  {
    EXPECT_NE(std::find(names.begin(), names.end(), expected), names.end())
      << "Missing: " << expected;
  }

  // Initial values: identity quaternion, all others 0
  for (auto & iface : ifaces) {
    double val = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(iface.get_value(val, true));
    const double expected = (iface.get_interface_name() == "orientation.w") ? 1.0 : 0.0;
    EXPECT_DOUBLE_EQ(val, expected) << iface.get_interface_name();
  }
}

// ── lifecycle + read (mock mode) ──────────────────────────────────────────────

class MockHwTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hw_ = std::make_unique<bno08x_hardware_interface::BNO08XHardwareInterface>();
    auto info = make_valid_imu_info();
    info.hardware_parameters["enable_mock_mode"] = "true";
    ASSERT_EQ(hw_->on_init(info), CallbackReturn::SUCCESS);
  }

  double get(const std::string & name)
  {
    for (auto & iface : ifaces_) {
      if (iface.get_interface_name() == name) {
        double v = std::numeric_limits<double>::quiet_NaN();
        (void)iface.get_value(v, true);
        return v;
      }
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  std::unique_ptr<bno08x_hardware_interface::BNO08XHardwareInterface> hw_;
  std::vector<hardware_interface::StateInterface> ifaces_;
  const rclcpp::Time     kTime{0, 0, RCL_ROS_TIME};
  const rclcpp::Duration kPeriod{0, static_cast<int32_t>(10e6)};
};

TEST_F(MockHwTest, FullLifecycle)
{
  EXPECT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_activate(inactive_state()),      CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_deactivate(active_state()),      CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_cleanup(inactive_state()),       CallbackReturn::SUCCESS);
}

TEST_F(MockHwTest, ReconfigureAfterCleanup)
{
  ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_cleanup(inactive_state()),       CallbackReturn::SUCCESS);

  hw_ = std::make_unique<bno08x_hardware_interface::BNO08XHardwareInterface>();
  auto info = make_valid_imu_info();
  info.hardware_parameters["enable_mock_mode"] = "true";
  ASSERT_EQ(hw_->on_init(info), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
}

TEST_F(MockHwTest, SameObjectReconfigureCycle)
{
  // ros2_control reuses the same plugin instance across lifecycle transitions.
  // Verify that the same hw_ object can be configured, cleaned up, and
  // configured again without creating a new instance.
  ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(inactive_state()),      CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_deactivate(active_state()),      CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_cleanup(inactive_state()),       CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
}

TEST_F(MockHwTest, ReadOutputsValid)
{
  ifaces_ = hw_->export_state_interfaces();
  ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(inactive_state()),      CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->read(kTime, kPeriod), return_type::OK);

  // Mock: identity quaternion, zeros elsewhere — all finite and non-NaN
  EXPECT_DOUBLE_EQ(get("orientation.w"), 1.0);
  EXPECT_DOUBLE_EQ(get("orientation.x"), 0.0);
  EXPECT_DOUBLE_EQ(get("orientation.y"), 0.0);
  EXPECT_DOUBLE_EQ(get("orientation.z"), 0.0);
  for (auto & iface : ifaces_) {
    double v = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(iface.get_value(v, true));
    EXPECT_FALSE(std::isnan(v)) << iface.get_interface_name();
    EXPECT_TRUE(std::isfinite(v)) << iface.get_interface_name();
  }

  // Quaternion unit norm
  const double qw = get("orientation.w"), qx = get("orientation.x"),
    qy = get("orientation.y"), qz = get("orientation.z");
  EXPECT_NEAR(std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz), 1.0, 0.05);
}

TEST_F(MockHwTest, MultipleReadsRemainStable)
{
  ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(inactive_state()),      CallbackReturn::SUCCESS);
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(hw_->read(kTime, kPeriod), return_type::OK) << "iteration " << i;
  }
}

TEST_F(MockHwTest, StateResetAfterCleanup)
{
  // Export interfaces before configuring so they stay valid throughout.
  ifaces_ = hw_->export_state_interfaces();
  ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(inactive_state()),      CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->read(kTime, kPeriod),               return_type::OK);

  // Cleanup should reset all state doubles to their initial values.
  ASSERT_EQ(hw_->on_cleanup(inactive_state()), CallbackReturn::SUCCESS);

  EXPECT_DOUBLE_EQ(get("orientation.w"), 1.0);
  EXPECT_DOUBLE_EQ(get("orientation.x"), 0.0);
  EXPECT_DOUBLE_EQ(get("orientation.y"), 0.0);
  EXPECT_DOUBLE_EQ(get("orientation.z"), 0.0);
  EXPECT_DOUBLE_EQ(get("angular_velocity.x"),    0.0);
  EXPECT_DOUBLE_EQ(get("angular_velocity.y"),    0.0);
  EXPECT_DOUBLE_EQ(get("angular_velocity.z"),    0.0);
  EXPECT_DOUBLE_EQ(get("linear_acceleration.x"), 0.0);
  EXPECT_DOUBLE_EQ(get("linear_acceleration.y"), 0.0);
  EXPECT_DOUBLE_EQ(get("linear_acceleration.z"), 0.0);
}

TEST_F(MockHwTest, ShutdownFromInactive)
{
  ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_shutdown(inactive_state()),      CallbackReturn::SUCCESS);
}

TEST_F(MockHwTest, ShutdownFromActive)
{
  ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(inactive_state()),      CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_shutdown(active_state()),        CallbackReturn::SUCCESS);
}

TEST_F(MockHwTest, OnErrorCleansUpAndAllowsRecovery)
{
  // on_error() must close hardware and return SUCCESS (→ UNCONFIGURED) so that
  // the controller manager can reconfigure without a process restart.
  // The base-class default returns ERROR (→ FINALIZED), leaking the I2C fd.
  ifaces_ = hw_->export_state_interfaces();
  ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(inactive_state()),      CallbackReturn::SUCCESS);

  EXPECT_EQ(hw_->on_error(active_state()), CallbackReturn::SUCCESS);

  // State must be reset to initial values (close_hardware called by on_error)
  EXPECT_DOUBLE_EQ(get("orientation.w"), 1.0);
  EXPECT_DOUBLE_EQ(get("orientation.x"), 0.0);
  EXPECT_DOUBLE_EQ(get("angular_velocity.x"),    0.0);
  EXPECT_DOUBLE_EQ(get("linear_acceleration.x"), 0.0);

  // Recovery: on_configure must succeed after on_error
  EXPECT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
