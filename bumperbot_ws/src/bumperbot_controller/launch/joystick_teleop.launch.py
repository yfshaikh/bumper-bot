#!/usr/bin/env python3
# =============================================================================
# joystick_teleop.launch.py  —  bumperbot_controller
# =============================================================================
# Starts the joystick teleoperation stack. Two nodes are launched:
#   1. joy_node     — reads the gamepad hardware and publishes sensor_msgs/Joy
#   2. joy_teleop   — maps Joy messages to Twist commands on cmd_vel
#
# Configuration files in config/ control button/axis mappings and speed limits.
# This file is included by both real_robot.launch.py (use_sim_time=False) and
# simulated_robot.launch.py (use_sim_time=True).
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Whether to use Gazebo /clock (sim) or wall clock (real robot)
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )

    # joy_teleop — translates sensor_msgs/Joy into velocity commands.
    # joy_teleop.yaml defines axis-to-velocity mappings and max speeds.
    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("bumperbot_controller"), "config", "joy_teleop.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # joy_node — reads from the gamepad device (/dev/input/js*).
    # joy_config.yaml specifies the device path and deadzone settings.
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("bumperbot_controller"), "config", "joy_config.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            joy_teleop,
            joy_node
        ]
    )
