#!/usr/bin/env python3
# =============================================================================
# kalman_filter.py  —  bumperbot_localization
# =============================================================================
# ROS 2 Python node implementing a 1-D Kalman filter for angular velocity.
# Python variant — the C++ version (kalman_filter.cpp) is also available.
#
# Purpose:
#   Fuse noisy wheel-encoder angular velocity with IMU gyroscope angular
#   velocity to produce a cleaner estimate of the robot's yaw rate.
#
# Inputs:
#   /bumperbot_controller/odom_noisy  (Odometry)  — from noisy_controller
#   /imu/out                          (Imu)        — from mpu6050_driver
#
# Output:
#   /bumperbot_controller/odom_kalman (Odometry)  — same as odom_noisy but
#   with twist.twist.angular.z replaced by the Kalman-filtered yaw rate.
#
# Algorithm (1-D Gaussian belief tracking):
#   1. State Prediction  — shift mean by motion increment, increase variance
#      by motion noise to reflect uncertainty growth over time.
#   2. Measurement Update — blend IMU measurement with predicted state;
#      trusted more when sensor variance < state variance.
# =============================================================================

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class KalmanFilter(Node):

    def __init__(self):
        super().__init__("kalman_filter")

        # Subscribe to the noisy wheel-encoder odometry
        self.odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom_noisy", self.odomCallback, 10)
        # Subscribe to raw IMU — we only use angular_velocity.z (yaw rate)
        self.imu_sub_ = self.create_subscription(Imu, "imu/out", self.imuCallback, 10)
        # Publish the filtered odometry (all fields copied from odom_noisy,
        # angular.z replaced by the Kalman estimate)
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom_kalman", 10)
        
        # Kalman state: Gaussian belief over angular velocity
        self.mean_ = 0.0       # current best estimate of angular velocity (rad/s)
        self.variance_ = 1000.0  # initially very uncertain — let measurements dominate

        # Noise parameters — tuned to trust IMU more than wheel odometry
        self.motion_variance_ = 4.0      # how much uncertainty grows per prediction step
        self.measurement_variance_ = 0.5 # how noisy the IMU measurement is (lower = more trusted)

        # Latest IMU angular velocity reading (updated asynchronously)
        self.imu_angular_z_ = 0.0

        # Skip the first odometry message — we need two to compute a delta
        self.is_first_odom_ = True
        self.last_angular_z_ = 0.0  # angular velocity from the previous odom message
        self.motion_ = 0.0          # change in angular velocity between odom callbacks

        # Output message; copied from odom_noisy with angular.z overwritten
        self.kalman_odom_ = Odometry()


    def odomCallback(self, odom):
        """Main filter loop — called each time a new noisy odometry arrives."""
        self.kalman_odom_ = odom  # copy all pose and velocity fields

        if self.is_first_odom_:
            # Bootstrap: initialise the filter state from the first measurement
            self.last_angular_z_ = odom.twist.twist.angular.z
            self.is_first_odom_ = False
            self.mean_ = odom.twist.twist.angular.z
            return
        
        # Compute how much the angular velocity changed since the last callback
        self.motion_ = odom.twist.twist.angular.z - self.last_angular_z_

        # Step 1: predict new state based on motion (uncertainty grows)
        self.statePrediction()
        # Step 2: correct prediction using the IMU measurement
        self.measurementUpdate()

        # Save current angular velocity as baseline for next delta
        self.last_angular_z_ = odom.twist.twist.angular.z

        # Replace the noisy angular velocity with the filtered estimate
        self.kalman_odom_.twist.twist.angular.z = self.mean_
        self.odom_pub_.publish(self.kalman_odom_)


    def imuCallback(self, imu):
        """Cache the latest IMU yaw-rate measurement for use in measurementUpdate."""
        self.imu_angular_z_ = imu.angular_velocity.z


    def measurementUpdate(self):
        """Correct the predicted state using the IMU measurement (Bayes update).

        Kalman measurement update equations:
          mean     = (σ_meas * μ + σ_state * z) / (σ_state + σ_meas)
          variance = (σ_state * σ_meas)         / (σ_state + σ_meas)

        A low measurement_variance_ (trusted sensor) pulls the mean strongly
        toward the IMU reading and shrinks the state variance.
        """
        self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_angular_z_) \
                   / (self.variance_ + self.measurement_variance_)
                     
        self.variance_ = (self.variance_ * self.measurement_variance_) \
                       / (self.variance_ + self.measurement_variance_)


    def statePrediction(self):
        """Advance the state estimate using the wheel-encoder motion model.

        Kalman state prediction equations:
          mean     = mean + motion
          variance = variance + motion_variance

        Adding motion_variance_ increases the uncertainty because encoder
        readings are noisy and we can't perfectly predict the new angular rate.
        """
        self.mean_ = self.mean_ + self.motion_
        self.variance_ = self.variance_ + self.motion_variance_


def main():
    rclpy.init()

    kalman_filter = KalmanFilter()
    rclpy.spin(kalman_filter)
    
    kalman_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()