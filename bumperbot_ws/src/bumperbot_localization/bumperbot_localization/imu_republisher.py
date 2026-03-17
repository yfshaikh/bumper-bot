#!/usr/bin/env python3
# =============================================================================
# imu_republisher.py  —  bumperbot_localization
# =============================================================================
# Thin relay node that bridges the raw IMU topic into the EKF pipeline.
#
# Problem: robot_localization's ekf_node expects all sensor inputs to use the
# same coordinate frame as the robot's EKF base frame ("base_footprint_ekf").
# The raw /imu/out messages from mpu6050_driver.py use "base_footprint" as
# their frame_id, which doesn't match.
#
# Solution: subscribe to /imu/out, swap the frame_id to "base_footprint_ekf",
# and republish on /imu_ekf — the topic ekf_node actually subscribes to.
#
# Note: The C++ equivalent (imu_republisher.cpp) does the same thing and is
# selected by default (use_python:=False) in local_localization.launch.py.
# =============================================================================

import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Global publisher reference — kept outside the class because the bare-function
# callback style requires access without a self reference
imu_pub = None

def imuCallback(imu):
    """Receive raw IMU, re-stamp with EKF frame ID, and republish."""
    global imu_pub
    # Only the frame_id changes; all sensor values pass through unchanged
    imu.header.frame_id = "base_footprint_ekf"
    imu_pub.publish(imu)


def main(args=None):
    global imu_pub
    rclpy.init(args=args)
    node = Node('imu_republisher_node')
    # Brief sleep to allow other nodes (especially the IMU driver) to
    # come up and start publishing before we begin forwarding
    time.sleep(1)
    # Republish on imu_ekf — the topic name expected by ekf.yaml
    imu_pub = node.create_publisher(Imu, "imu_ekf", 10)
    # Subscribe to the raw IMU topic from mpu6050_driver (or Gazebo bridge)
    imu_sub = node.create_subscription(Imu, "imu/out", imuCallback, 10)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
