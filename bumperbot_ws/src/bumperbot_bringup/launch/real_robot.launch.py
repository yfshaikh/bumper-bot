#!/usr/bin/env python3
# =============================================================================
# real_robot.launch.py  —  bumperbot_bringup
# =============================================================================
# Top-level launch file for the PHYSICAL robot.
# Composes four sub-launches/nodes into a single bringup command:
#   1. hardware_interface  — opens serial to Arduino, loads ros2_control
#   2. controller          — spawns noisy_controller (C++) + joint_state_broadcaster
#   3. joystick_teleop     — reads gamepad and publishes cmd_vel
#   4. mpu6050_driver      — reads IMU over I2C and publishes /imu/out
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Hardware Interface ---
    # Starts controller_manager + BumperbotInterface (Arduino serial plugin).
    # Reads wheel encoder feedback and sends velocity commands over serial.
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )
    
    # --- Motion Controller ---
    # use_simple_controller=False  → uses bumperbot_controller (ros2_control velocity controller)
    # use_python=False             → uses C++ noisy_controller executable
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )
    
    # --- Joystick Teleoperation ---
    # use_sim_time=False → uses wall-clock time (real robot, not simulation)
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    # --- IMU Driver ---
    # Standalone Python node that reads MPU6050 over I2C at 100 Hz
    # and publishes sensor_msgs/Imu on /imu/out.
    imu_driver_node = Node(
        package="bumperbot_firmware",
        executable="mpu6050_driver.py"
    )
    
    return LaunchDescription([
        hardware_interface,
        controller,
        joystick,
        imu_driver_node,
    ])