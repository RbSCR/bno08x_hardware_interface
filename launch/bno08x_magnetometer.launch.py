#!/usr/bin/env python3
"""
Launch file for the BNO08x IMU hardware interface.

Starts the complete ros2_control stack for the BNO08x IMU, including:
- Robot state publisher for TF transforms
- Controller manager with the BNO08x SensorInterface hardware plugin
- IMU sensor broadcaster publishing sensor_msgs/Imu to /imu_sensor_broadcaster/imu
- Magnetometer broadcaster publishing sensor_msgs/MagneticField

This is a convenience launch file. The same functionality can be achieved
by using the 'bno08x.launch.py' file with the parameters 'enable_magnetometer:=true'
and 'broadcast_magnetometer:=true'
and -if the magnetometer values should be broadcasted - controller config-yaml with a
reference to the magnetometer-broadcaster.

Base usage:
    ros2 launch bno08x_hardware_interface bno08x_magnetometer.launch.py

    See the declared_arguments below for the default values.

Example (other) usage:
    <base-usage> i2c_device:=/dev/i2c-bno08-B i2c_addr:=4B
    <base-usage> axis_remap:=North-West-Up
    <base-usage> enable_mock_mode:=true
    <base-usage> publish_tf:=false
    <base-usage> broadcast_magnetometer:=false
    <base-usage> publish_diagnostics:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'i2c_device',
            default_value='/dev/i2c-bno08x',
            description='I2C device name (e.g. /dev/i2c-bno08x)'
        ),
        DeclareLaunchArgument(
            'i2c_addr',
            default_value='4A',
            description='I2C device address in hex without 0x prefix (default: 4A = 0x4A, alternative: 4B)'
        ),
        DeclareLaunchArgument(
            'axis_remap',
            default_value='East-North-Up',
            description='BNO08X axis placement configuration: valid combination of North East South West Up Down'
                'in the format <xxx>-<xxx>-<xxx>'
                'See datasheet Figure 4-3  page 41 for the valid combinations.'
        ),
        DeclareLaunchArgument(
            'imu_rate',
            default_value='100',
            description='IMU sensor rate in Hz.'
                'See datasheet Figure 6-16  page 50.'
        ),
        DeclareLaunchArgument(
            'enable_magnetometer',
            default_value='true',
            description='Enable magnetometer in hardware_interface'
        ),
        DeclareLaunchArgument(
            'magnetometer_rate',
            default_value='100',
            description='Magnetometer rate in Hz.'
                'See datasheet Figure 6-16  page 50.'
        ),
        DeclareLaunchArgument(
            'enable_mock_mode',
            default_value='false',
            description='Use mock/simulation mode (no hardware required)'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description=(
                'Publish a dynamic world→base_link TF from IMU orientation for RViz visualization'
            ),
        ),
        DeclareLaunchArgument(
            'broadcast_magnetometer',
            default_value='true',
            description=(
                'Broadcast magnetic orientation'
                'Note: to be usefull also set enable_magnetometer to true'
            ),
        ),

        DeclareLaunchArgument(
            # TODO(rbscr) diagnostics temporarily default disabled
            'publish_diagnostics',
            default_value='false',
            description=(
                'Run the bno08x_diagnostics companion node to publish sensor health '
                'and calibration status to /diagnostics at 1 Hz'
            ),
        ),
    ]

    i2c_device = LaunchConfiguration('i2c_device')
    i2c_addr = LaunchConfiguration('i2c_addr')
    axis_remap = LaunchConfiguration('axis_remap')
    imu_rate = LaunchConfiguration('imu_rate')
    enable_magnetometer = LaunchConfiguration('enable_magnetometer')
    magnetometer_rate = LaunchConfiguration('magnetometer_rate')
    enable_mock = LaunchConfiguration('enable_mock_mode')
    publish_tf = LaunchConfiguration('publish_tf')
    broadcast_magnetometer = LaunchConfiguration('broadcast_magnetometer')
    publish_diagnostics = LaunchConfiguration('publish_diagnostics')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('bno08x_hardware_interface'), 'config', 'bno08x_magnetometer.urdf.xacro']
            ),
            ' ',
            'i2c_device:=', i2c_device,
            ' ',
            'i2c_addr:=', i2c_addr,
            ' ',
            'axis_remap:=', axis_remap,
            ' ',
            'imu_rate:=', imu_rate,
            ' ',
            'enable_magnetometer:=', enable_magnetometer,
            ' ',
            'magnetomeer_rate:=', magnetometer_rate,
            ' ',
            'enable_mock_mode:=', enable_mock,
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Controller configuration
    controller_config = PathJoinSubstitution(
        [FindPackageShare('bno08x_hardware_interface'), 'config', 'imu_magnetometer_broadcaster.yaml']
    )

    # Controller manager (ros2_control_node)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[robot_description, controller_config],
    )

    # IMU sensor broadcaster spawner
    imu_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Optional: Magnetometer broadcaster spawner
    magnetometer_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['magnetometer_broadcaster', '--controller-manager', '/controller_manager'],
        condition=IfCondition(broadcast_magnetometer)
    )

    # Optional: relay IMU orientation to TF for RViz 3D visualization.
    # Set fixed frame to 'world' in RViz to see the sensor orientation animate.
    imu_tf_broadcaster_node = Node(
        package='bno08x_hardware_interface',
        executable='imu_tf_broadcaster',
        name='imu_tf_broadcaster',
        output='screen',
        condition=IfCondition(publish_tf),
    )

    # TODO(rbscr) diagnostics_node nog maken (of in hardware_interface opnemen)
    # Optional: publish sensor health and calibration status to /diagnostics at 1 Hz.
    # Compatible with rqt_robot_monitor and diagnostic_aggregator.
    bno08x_diagnostics_node = Node(
        package='bno08x_hardware_interface',
        executable='bno08x_diagnostics',
        name='bno8x_diagnostics',
        output='screen',
        parameters=[{
            'i2c_device':  i2c_device,
            'i2c_addr':    ParameterValue(i2c_addr, value_type=str),
            'enable_mock_mode': ParameterValue(enable_mock, value_type=str),
        }],
        condition=IfCondition(publish_diagnostics),
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            controller_manager_node,
            imu_broadcaster_spawner,
            magnetometer_broadcaster_spawner,
            imu_tf_broadcaster_node,
            bno08x_diagnostics_node,
        ]
    )
