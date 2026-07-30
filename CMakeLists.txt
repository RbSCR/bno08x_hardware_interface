cmake_minimum_required(VERSION 3.20)
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

install(TARGETS bno08x_hardware_interface
  DESTINATION lib/${PROJECT_NAME})

install(FILES bno08x_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
