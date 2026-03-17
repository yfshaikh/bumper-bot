#!/usr/bin/env python3
# =============================================================================
# simple_controller.py  —  bumperbot_controller
# =============================================================================
# ROS 2 Python node implementing a CLEAN differential-drive controller.
# Used when use_simple_controller:=True (simulation / tutorial mode only).
# NOT used on the real robot (real robot uses noisy_controller.cpp).
#
# Responsibilities:
#   1. Forward kinematics  — converts desired robot v/w into per-wheel speeds
#      and publishes them to /simple_velocity_controller/commands.
#   2. Inverse kinematics  — integrates encoder deltas from /joint_states into
#      an (x, y, θ) pose and publishes it as /bumperbot_controller/odom.
#   3. TF broadcast        — sends the odom → base_footprint transform.
# =============================================================================

import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler


class SimpleController(Node):

    def __init__(self):
        super().__init__("simple_controller")

        # Declare parameters so they can be overridden via launch arguments
        self.declare_parameter("wheel_radius", 0.033)       # metres
        self.declare_parameter("wheel_separation", 0.17)    # metres

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius %d" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation %d" % self.wheel_separation_)

        # Previous wheel positions tracked for delta-position odometry
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        # Accumulated robot pose in the odom frame
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Publisher: per-wheel speed commands for the simple_velocity_controller
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        # Subscriber: desired robot velocity from joystick / navigation stack
        self.vel_sub_ = self.create_subscription(TwistStamped, "bumperbot_controller/cmd_vel", self.velCallback, 10)
        # Subscriber: actual wheel positions from joint_state_broadcaster
        self.joint_sub_ = self.create_subscription(JointState,"joint_states", self.jointCallback, 10)
        # Publisher: computed odometry (pose + velocity in the odom frame)
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom", 10)

        # Speed conversion matrix M maps [v, w] → [ω_left, ω_right].
        # Derived from the differential-drive kinematic equations:
        #   v = r*(ω_r + ω_l)/2,  w = r*(ω_r - ω_l)/d
        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_)

        # Pre-fill the invariant fields of the odometry message to avoid
        # re-setting them on every callback invocation
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # TF broadcaster sends the odom → base_footprint transform every cycle
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

        self.prev_time_ = self.get_clock().now()


    def velCallback(self, msg):
        """Forward kinematics: convert (v, w) → per-wheel angular speeds.

        Inverts the speed_conversion_ matrix M so that:
            [ω_right, ω_left] = M⁻¹ * [v, w]
        Then publishes the two wheel speed targets.
        """
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        # Invert the kinematic matrix to get individual wheel speeds
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) 

        wheel_speed_msg = Float64MultiArray()
        # data[0] = right wheel, data[1] = left wheel (controller convention)
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]

        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    
    def jointCallback(self, msg):
        """Inverse kinematics: integrate encoder deltas → pose and velocity.

        Given angular displacements of both wheels since the last callback:
          1. Compute each wheel's angular velocity (fi = dp / dt).
          2. Compute robot linear and angular velocity from the kinematic model.
          3. Integrate to update (x, y, θ) pose.
          4. Publish Odometry and broadcast TF.
        """
        # Change in wheel angle (radians) since last callback
        dp_left = msg.position[0] - self.left_wheel_prev_pos_
        dp_right = msg.position[1] - self.right_wheel_prev_pos_
        # Elapsed time — use message timestamp for deterministic integration
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        # Store current values as "previous" for the next iteration
        self.left_wheel_prev_pos_ = msg.position[0]
        self.right_wheel_prev_pos_ = msg.position[1]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        # Angular velocity of each wheel (rad/s)
        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        # Robot body linear velocity (m/s) and angular velocity (rad/s)
        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_

        # Pose increment in the robot's local frame, then rotate to odom frame
        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2  # arc length
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)
        
        # Convert yaw angle to quaternion for the message
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

        # Broadcast TF: odom → base_footprint so the rest of the system knows
        # where the robot is relative to its starting position
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

    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()