#!/usr/bin/env python3
# =============================================================================
# hardware_interface.launch.py  —  bumperbot_firmware
# =============================================================================
# Starts the ros2_control infrastructure for the PHYSICAL robot.
# Two nodes are launched:
#   1. robot_state_publisher — parses the URDF (with is_sim:=False so the
#      BumperbotInterface plugin is selected) and publishes the static TF tree.
#   2. controller_manager   — loads BumperbotInterface, opens the Arduino serial
#      port, and exposes the hardware command/state interfaces to the rest of
#      the controller stack.
# This file is included by bumperbot_bringup/launch/real_robot.launch.py.
# =============================================================================

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Process the Xacro file into a plain URDF string at launch time.
    # is_sim:=False tells bumperbot_ros2_control.xacro to load the real
    # BumperbotInterface serial plugin instead of the Gazebo mock plugin.
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("bumperbot_description"),
                    "urdf",
                    "bumperbot.urdf.xacro",
                ),
                " is_sim:=False"   # selects the real hardware interface
            ]
        ),
        value_type=str,
    )

    # Publishes geometry transforms (TF) derived from the URDF joint tree so
    # that other nodes (e.g. rviz, localization) know where each link is.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # ros2_control node — loads controller plugins and the hardware interface.
    # bumperbot_controllers.yaml defines which controllers to load
    # (joint_state_broadcaster, bumperbot_controller velocity controller).
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},     # wall clock, not Gazebo /clock
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "config",
                "bumperbot_controllers.yaml",   # PID gains and controller types
            ),
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            controller_manager,
        ]
    )