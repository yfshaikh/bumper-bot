// =============================================================================
// kalman_filter.cpp  —  bumperbot_localization
// =============================================================================
// C++ implementation of a 1-D Kalman filter for angular velocity fusion.
// Selected by default (use_python:=False) in local_localization.launch.py.
// Python equivalent: bumperbot_localization/kalman_filter.py
//
// Fuses two sources of angular velocity (yaw rate) information:
//   - Wheel encoder odometry (/bumperbot_controller/odom_noisy) — motion model
//   - IMU gyroscope (/imu/out)                                  — measurement
//
// See kalman_filter.hpp for the full algorithm and parameter description.
// =============================================================================

#include "bumperbot_localization/kalman_filter.hpp"

using std::placeholders::_1;

KalmanFilter::KalmanFilter(const std::string &name)
    : Node(name), mean_(0.0), // initial angular velocity estimate: 0 rad/s
      variance_(
          1000.0), // high initial uncertainty → let first measurement dominate
      motion_variance_(4.0), // process noise (constant — tunable)
      measurement_variance_(
          0.5), // IMU noise (lower = more trusted than encoder)
      motion_(0.0),
      is_first_odom_(true), // bootstrap flag — skip first callback
      last_angular_z_(0.0), imu_angular_z_(0.0) {
  // Subscribe to noisy encoder odometry — drives the prediction step
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "bumperbot_controller/odom_noisy", 10,
      std::bind(&KalmanFilter::odomCallback, this, _1));
  // Subscribe to raw IMU — cache for the measurement update step
  // Large queue (1000) to tolerate IMU arriving faster than odom
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/out", 1000, std::bind(&KalmanFilter::imuCallback, this, _1));
  // Publish filtered odometry — same as odom_noisy but angular.z replaced
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "bumperbot_controller/odom_kalman", 10);
}

void KalmanFilter::odomCallback(const nav_msgs::msg::Odometry &odom) {
  // Copy the full odom message — we'll overwrite only angular.z at the end
  kalman_odom_ = odom;

  // Bootstrap: on the very first message, initialise the filter state and
  // return
  if (is_first_odom_) {
    last_angular_z_ = odom.twist.twist.angular.z;
    mean_ =
        odom.twist.twist.angular.z; // seed the filter with the first reading
    is_first_odom_ = false;
    return;
  }

  // Motion = change in angular velocity since the last odom callback.
  // This drives the prediction step (the "u" term in the filter).
  motion_ = odom.twist.twist.angular.z - last_angular_z_;

  // Run one full Kalman cycle: predict then correct
  statePrediction();
  measurementUpdate();

  // Save current reading as baseline for the next delta
  last_angular_z_ = odom.twist.twist.angular.z;

  // Publish the corrected odometry with the filtered angular.z
  kalman_odom_.twist.twist.angular.z = mean_;
  odom_pub_->publish(kalman_odom_);
}

void KalmanFilter::imuCallback(const sensor_msgs::msg::Imu &imu) {
  // Cache the latest IMU yaw rate for use in measurementUpdate().
  // The IMU publishes at a different rate than odom, so we just store
  // the most recent value and use it on the next odom callback.
  imu_angular_z_ = imu.angular_velocity.z;
}

void KalmanFilter::measurementUpdate() {
  // Bayes update — blend the prediction with the IMU measurement.
  // mean  = (σ_sensor · μ_pred + σ_pred · z_imu) / (σ_pred + σ_sensor)
  // var   = (σ_pred · σ_sensor)                  / (σ_pred + σ_sensor)
  //
  // The IMU (σ=0.5) is trusted more than the encoder model (σ can grow large),
  // so the filtered mean is pulled toward imu_angular_z_ each cycle.
  mean_ = (measurement_variance_ * mean_ + variance_ * imu_angular_z_) /
          (variance_ + measurement_variance_);

  variance_ =
      (variance_ * measurement_variance_) / (variance_ + measurement_variance_);
}

void KalmanFilter::statePrediction() {
  // Apply the motion model: advance the mean by the encoder-derived motion
  // and inflate the variance to reflect growing uncertainty.
  // Adding motion_variance_ models the fact that wheel odometry is imperfect.
  mean_ = mean_ + motion_;
  variance_ = variance_ + motion_variance_;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KalmanFilter>("kalman_filter");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}