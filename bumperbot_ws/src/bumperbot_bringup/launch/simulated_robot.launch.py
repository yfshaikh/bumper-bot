#!/usr/bin/env python3
# =============================================================================
# simulated_robot.launch.py  —  bumperbot_bringup
# =============================================================================
# Top-level launch file for the GAZEBO SIMULATION (not used on real hardware).
# Composes three sub-launches:
#   1. gazebo          — spawns bumperbot in a Gazebo world with sim hardware plugins
#   2. controller      — spawns noisy_controller (C++) using sim time
#   3. joystick_teleop — reads gamepad and publishes cmd_vel with sim time
# The IMU driver (mpu6050_driver.py) is NOT started here because the IMU is
# simulated by the Gazebo bridge (ros_gz_bridge) instead.
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Gazebo Simulation ---
    # Starts Gazebo, spawns the robot URDF, and bridges /clock + /imu topics
    # from Gazebo into ROS 2 topics.
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    # --- Motion Controller ---
    # Same controller launch as the real robot, but use_python=False (C++) and
    # use_simple_controller=False (velocity controller). Sim time is inherited
    # from the Gazebo /clock bridge.
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
    # use_sim_time=True → controller timestamps match Gazebo /clock, not wall clock
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )
    
    return LaunchDescription([
        gazebo,
        controller,
        joystick,
    ])