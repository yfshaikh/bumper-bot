// =============================================================================
// kalman_filter.hpp  —  bumperbot_localization
// =============================================================================
// Header for the KalmanFilter ROS 2 node (C++ version).
// Python variant: bumperbot_localization/kalman_filter.py
//
// Implements a 1-D Kalman filter that fuses noisy wheel-encoder angular
// velocity with IMU gyroscope angular velocity to produce a cleaner yaw-rate
// estimate. Only the angular.z field of the odometry output is filtered —
// all other pose and velocity fields are copied unchanged from odom_noisy.
//
// Filter state (Gaussian belief):
//   mean_     — best estimate of angular velocity (rad/s)
//   variance_ — uncertainty of that estimate (starts at 1000 — very uncertain)
//
// Algorithm every odom callback:
//   1. statePrediction():  mean += motion;  variance += motion_variance_
//   2. measurementUpdate(): Bayesian update using IMU yaw rate as measurement
// =============================================================================

#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <nav_msgs/msg/odometry.hpp> // input: odom_noisy; output: odom_kalman
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp> // input: /imu/out (only angular_velocity.z used)

class KalmanFilter : public rclcpp::Node {
public:
  // Constructs the node and sets up subscriptions and publisher
  KalmanFilter(const std::string &name);

  // Step 1: advance state estimate using wheel encoder motion model
  void statePrediction();

  // Step 2: correct estimate using IMU angular velocity measurement
  void measurementUpdate();

private:
  // Triggered on each noisy odom message — runs prediction + update cycle
  void odomCallback(const nav_msgs::msg::Odometry &);

  // Triggered on each IMU message — caches the latest gyro z reading
  void imuCallback(const sensor_msgs::msg::Imu &);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odom_sub_;                                                   // odom_noisy
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_; // /imu/out
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      odom_pub_; // odom_kalman

  // --- Kalman filter state ---
  double mean_;            // current angular velocity estimate (rad/s)
  double variance_;        // uncertainty of the estimate
  double motion_variance_; // process noise: uncertainty added per prediction
                           // step (4.0)
  double measurement_variance_; // IMU measurement noise (0.5 = trusted more
                                // than encoder)
  double motion_; // delta angular velocity between consecutive odom callbacks

  bool is_first_odom_; // skip the first message (need two to compute a delta)
  double last_angular_z_; // angular velocity from the previous odom callback
  double imu_angular_z_;  // latest IMU yaw rate, updated asynchronously

  nav_msgs::msg::Odometry kalman_odom_; // output message, copied from
                                        // odom_noisy with angular.z overwritten
};

#endif // KALMAN_FILTER_HPP