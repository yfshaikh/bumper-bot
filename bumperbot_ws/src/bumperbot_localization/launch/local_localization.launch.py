#!/usr/bin/env python3
# =============================================================================
# local_localization.launch.py  —  bumperbot_localization
# =============================================================================
# Starts the full localization stack on top of the base bringup.
# NOT launched by real_robot.launch.py — must be started separately.
#
# Three things are launched:
#   1. static_transform_publisher — publishes a fixed TF from the EKF base
#      frame to the IMU link so robot_localization can use the IMU.
#   2. ekf_filter_node (robot_localization) — full-state EKF that fuses
#      wheel odometry and IMU into /odometry/filtered + TF odom→base_footprint_ekf
#   3. imu_republisher — relays /imu/out with the correct frame_id for EKF
#      (available as Python or C++ via use_python argument)
#
# Note: kalman_filter node is NOT included here; it is intended to be run
# separately as a standalone demonstration of 1-D sensor fusion.
# =============================================================================

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
import os

def generate_launch_description():

    # Whether to use Python or C++ for the imu_republisher node
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",   # C++ by default
    )

    use_python = LaunchConfiguration("use_python")

    # Publishes a fixed transform: base_footprint_ekf → imu_link_ekf
    # The z-offset (0.103 m) and 180° rotation around X match the real
    # IMU mounting position defined in the URDF imu_joint.
    # This lets robot_localization know where the IMU is in the EKF frame.
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0","--z", "0.103",
                   "--qx", "1", "--qy", "0", "--qz", "0", "--qw", "0",
                   "--frame-id", "base_footprint_ekf",
                   "--child-frame-id", "imu_link_ekf"],
    )

    # Extended Kalman Filter from the robot_localization package.
    # Reads ekf.yaml which specifies which odom and IMU topics to fuse
    # and what state variables to include from each sensor.
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("bumperbot_localization"), "config", "ekf.yaml")],
    )

    # Python variant of imu_republisher — selected when use_python:=True
    imu_republisher_py = Node(
        package="bumperbot_localization",
        executable="imu_republisher.py",
        condition=IfCondition(use_python),
    )

    # C++ variant of imu_republisher — selected when use_python:=False (default)
    imu_republisher_cpp = Node(
        package="bumperbot_localization",
        executable="imu_republisher",
        condition=UnlessCondition(use_python),
    )

    return LaunchDescription([
        use_python_arg,
        static_transform_publisher,
        robot_localization,
        imu_republisher_py,
        imu_republisher_cpp,   
    ])