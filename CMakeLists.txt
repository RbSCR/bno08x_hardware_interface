cmake_minimum_required(VERSION 3.28.3)
project(bno08x_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(bno08x_sh2_libraries REQUIRED)


# ── BNO08X Hardware Interface (ros2_control sensor plugin) ──────────────────
add_library(${PROJECT_NAME} SHARED
  src/bno08x_hardware_interface.cpp
)

target_include_directories(bno08x_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(bno08x_hardware_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(bno08x_hardware_interface
  ${bno08x_sh2_libraries_TARGETS}
  ${pluginlib_TARGETS}
  ${rclcpp_TARGETS}
  ${rclcpp_lifecycle_TARGETS}
  ${hardware_interface_TARGETS}
)

# Export hardware interface as plugin
pluginlib_export_plugin_description_file(hardware_interface bno08x_hardware_interface.xml)

# ── IMU TF broadcaster (optional visualization helper) ───────────────────────
add_executable(imu_tf_broadcaster
  src/imu_tf_broadcaster.cpp
)
target_link_libraries(imu_tf_broadcaster
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
)
install(TARGETS imu_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS bno08x_hardware_interface
  DESTINATION lib/${PROJECT_NAME})

install(FILES bno08x_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)

  # = = = Linting = = =
  find_package(ament_lint_auto REQUIRED)
  # Linters to be used in the 'auto-linting' are mentioned in package.xml

  ament_lint_auto_find_test_dependencies()

  # = = = Tests = = =
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)


  # C++ unit / integration tests
  # - on_init validation (no hardware)
  # - export_state_interfaces (no hardware)
  # - lifecycle transitions (real hardware when available, mock mode otherwise)
  # - read() sanity checks (real hardware when available, mock mode otherwise)
  ament_add_gtest(test_hal_hardware_interface
    test/test_hardware_interface.cpp
  )
  target_link_libraries(test_hal_hardware_interface
    ${PROJECT_NAME}
    ${rclcpp_TARGETS}
    hardware_interface::hardware_interface
  )
  target_include_directories(test_hal_hardware_interface PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${BNO08X_DRIVER_DIR}
  )

  # Python launch tests — full stack against real BNO08X hardware or mock mode.
  # Uses enable_mock_mode:=true when the sensor is absent so CI always runs all tests.

  # Tests without magnetometer broadcaster; IMU broadcaster is always spawned
  add_launch_test(test/test_bno08x.launch.py)

  # Tests with IMU broadcaster and magnetometer broadcaster
  add_launch_test(test/test_bno08x_magnetometer.launch.py)

  # Tests with 'fixed' hardware parameters and without magnetometer broadcaster; IMU broadcaster is always spawned
  add_launch_test(test/test_bno08x_fixedhwparams.launch.py)

endif()

ament_package()
