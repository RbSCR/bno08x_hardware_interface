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


# ── SH2 LIBRAY (kindof external submodule) ────────────────────
# Note sh2_library had it's own CmakeLists.txt (in include/sh2)
#      renamed it so it isn't used anymore
set(SH2_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/sh2)

add_library(sh2_library
  ${SH2_LIBRARY_DIR}/sh2.c
  ${SH2_LIBRARY_DIR}/sh2_SensorValue.c
  ${SH2_LIBRARY_DIR}/sh2_util.c
  ${SH2_LIBRARY_DIR}/shtp.c
)

target_include_directories(sh2_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SH2_LIBRARY_DIR}
)

target_compile_features(sh2_library PUBLIC c_std_99)

set_target_properties(sh2_library PROPERTIES POSITION_INDEPENDENT_CODE ON)

# copied from bno08x_driver
target_compile_options(sh2_library
  PUBLIC -Wno-sign-compare -Wno-unused-parameter -Wno-unused-variable
)

# ── BNO08x driver (kindof external submodule) ────────────────────
set(BNO08X_DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/bno08x_driver)

add_library(bno08x_driver
  ${BNO08X_DRIVER_DIR}/bno08x.cpp
)

target_include_directories(bno08x_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${BNO08X_DRIVER_DIR}
)

set_target_properties(bno08x_driver PROPERTIES POSITION_INDEPENDENT_CODE ON)

# copied from bno08x_driver
target_compile_options(bno08x_driver
  PUBLIC -Wno-sign-compare -Wno-unused-parameter -Wno-unused-variable
)

target_link_libraries(bno08x_driver
  sh2_library
)


# was: add_executable(bno08x_hardware_interface src/bno08x_hardware_interface.cpp)
# ── BNO08X Hardware Interface (ros2_control sensor plugin) ──────────────────
add_library(${PROJECT_NAME} SHARED
  src/bno08x_hardware_interface/bno08x_hardware_interface.cpp
)

target_include_directories(bno08x_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(bno08x_hardware_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(bno08x_hardware_interface
  sh2_library
  bno08x_driver
  ${pluginlib_TARGETS}
  ${rclcpp_TARGETS}
  ${rclcpp_lifecycle_TARGETS}
  ${hardware_interface_TARGETS}
)

# Export hardware interface as plugin
pluginlib_export_plugin_description_file(hardware_interface bno08x_hardware_interface.xml)

# ── IMU TF broadcaster (optional visualization helper) ───────────────────────
add_executable(imu_tf_broadcaster
  src/bno08x_hardware_interface/imu_tf_broadcaster.cpp
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

  # Linting disabled for 'external' files
  list(APPEND AMENT_LINT_AUTO_FILE_EXCLUDE
    "include/sh2/*"
    "include/bno08x_driver/*"
    "src/bno08x_driver/*"
  )

  ament_lint_auto_find_test_dependencies()

  # = = = Tests = = =
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # - - - bno08x_driver tests - - -
  ament_add_gmock(test_driver_i2c_mock
    test/bno08x_driver/i2c_interface_mock_test.cpp
  )
  if(TARGET test_driver_i2c_mock)
    target_include_directories(test_driver_i2c_mock PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  endif()

  ament_add_gmock(test_driver_watchdog test/bno08x_driver/watchdog_test.cpp)
  if(TARGET test_driver_watchdog)
    target_include_directories(test_driver_watchdog PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  endif()

  # - - - bno08x_hardware_interface tests - - -

  # C++ unit / integration tests
  # - on_init validation (no hardware)
  # - export_state_interfaces (no hardware)
  # - lifecycle transitions (real hardware when available, mock mode otherwise)
  # - read() sanity checks (real hardware when available, mock mode otherwise)
  ament_add_gtest(test_hal_hardware_interface
    test/bno08x_hardware_interface/test_hardware_interface.cpp
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
  add_launch_test(test/bno08x_hardware_interface/test_bno08x.launch.py)

  # Tests with IMU broadcaster and magnetometer broadcaster
  add_launch_test(test/bno08x_hardware_interface/test_bno08x_magnetometer.launch.py)

  # Tests with 'fixed' hardware parameters and without magnetometer broadcaster; IMU broadcaster is always spawned
  add_launch_test(test/bno08x_hardware_interface/test_bno08x_fixedhwparams.launch.py)

endif()

ament_package()
