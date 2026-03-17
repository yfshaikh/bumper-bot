#!/usr/bin/env python3
# =============================================================================
# noisy_controller.py  —  bumperbot_controller
# =============================================================================
# ROS 2 Python node implementing a NOISY differential-drive odometry estimator.
# Python variant — the C++ version (noisy_controller.cpp) is used on the real
# robot. This file can be selected with use_python:=True.
#
# Identical to simple_controller.py in structure, but deliberately adds
# Gaussian noise (σ=0.005 rad) to the raw wheel encoder readings before
# computing odometry. This simulates real-world encoder imperfections and
# calibration error so the localization stack (Kalman filter + EKF) has
# something meaningful to correct for.
#
# Publishes noisy odometry on /bumperbot_controller/odom_noisy (NOT /odom)
# so both clean and noisy estimates can coexist on the same system.
# =============================================================================

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler


class NoisyController(Node):

    def __init__(self):
        super().__init__("noisy_controller")

        # Declare parameters — can be overridden with intentional errors in
        # controller.launch.py to simulate imperfect wheel calibration
        self.declare_parameter("wheel_radius", 0.033)       # metres
        self.declare_parameter("wheel_separation", 0.17)    # metres

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius %d" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation %d" % self.wheel_separation_)

        # Previous encoder positions for delta integration
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        # Accumulated pose in the odom frame
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Subscriber: actual wheel positions from joint_state_broadcaster
        self.joint_sub_ = self.create_subscription(JointState,"joint_states", self.jointCallback, 10)
        # Publisher: noisy odometry — consumed by kalman_filter and ekf_node
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom_noisy", 10)

        # Speed conversion matrix (same as simple_controller, used in logging)
        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_)

        # Pre-fill invariant odometry fields
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        # child frame is base_footprint_ekf (not base_footprint) so EKF can
        # distinguish its input from the clean odometry
        self.odom_msg_.child_frame_id = "base_footprint_ekf"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # TF broadcaster for the noisy estimate frame
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        # Separate child frame so it doesn't collide with the clean odom TF
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"

        self.prev_time_ = self.get_clock().now()

    
    def jointCallback(self, msg):
        """Inverse kinematics with injected Gaussian noise.

        The key difference from simple_controller: before computing deltas,
        zero-mean Gaussian noise (σ=0.005 rad) is added to each encoder
        reading. This mimics encoder quantisation error and wheel slip.
        """
        # --- Inject noise ---
        # np.random.normal(mean, std) draws one sample from N(0, 0.005)
        wheel_encoder_left = msg.position[0] + np.random.normal(0, 0.005)
        wheel_encoder_right = msg.position[1] + np.random.normal(0, 0.005)

        # Angle deltas using the noise-corrupted readings
        dp_left = wheel_encoder_left - self.left_wheel_prev_pos_
        dp_right = wheel_encoder_right - self.right_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        # Store RAW (non-noisy) values as baseline for next delta calculation
        # so noise doesn't accumulate multiplicatively across callbacks
        self.left_wheel_prev_pos_ = msg.position[0]
        self.right_wheel_prev_pos_ = msg.position[1]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        # Angular velocity of each wheel (rad/s)
        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        # Robot body velocities from the differential-drive kinematic model
        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_

        # Noise-corrupted pose increment
        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)
        
        # Publish noisy odometry
        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        self.odom_pub_.publish(self.odom_msg_)

        # Broadcast TF: odom → base_footprint_noisy
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform_stamped_)


def main():
    rclpy.init()

    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)
    
    noisy_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()