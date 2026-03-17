// =============================================================================
// noisy_controller.cpp  —  bumperbot_controller
// =============================================================================
// C++ implementation of NoisyController — the differential-drive odometry
// estimator used on the REAL ROBOT (selected by real_robot.launch.py with
// use_simple_controller:=False, use_python:=False).
//
// Key difference from SimpleController:
//   Before computing encoder deltas, Gaussian noise is sampled from
//   std::normal_distribution<double>(0.0, 0.005) and added to each encoder
//   reading. This simulates real-world wheel-slip and encoder quantisation
//   error so the Kalman filter / EKF in bumperbot_localization has something
//   meaningful to correct.
//
// The intentional wheel geometry error (wheel_radius_error,
// wheel_separation_error passed from controller.launch.py) simulates imperfect
// calibration.
//
// See noisy_controller.hpp for the full class description.
// =============================================================================

#include "bumperbot_controller/noisy_controller.hpp"
#include <random>                      // std::normal_distribution for noise
#include <tf2/LinearMath/Quaternion.h> // tf2::Quaternion for yaw → quaternion

using std::placeholders::_1;

NoisyController::NoisyController(const std::string &name)
    : Node(name), left_wheel_prev_pos_(0.0), right_wheel_prev_pos_(0.0),
      x_(0.0), y_(0.0), theta_(0.0) {
  // Declare parameters — these are set with intentional error by
  // controller.launch.py e.g. wheel_radius = 0.033 + 0.005 = 0.038
  // (deliberately wrong)
  declare_parameter("wheel_radius", 0.033);
  declare_parameter("wheel_separation", 0.17);
  wheel_radius_ = get_parameter("wheel_radius").as_double();
  wheel_separation_ = get_parameter("wheel_separation").as_double();
  RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius " << wheel_radius_);
  RCLCPP_INFO_STREAM(get_logger(),
                     "Using wheel separation " << wheel_separation_);

  // Subscribe to encoder positions — written by the hardware interface read()
  // cycle
  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&NoisyController::jointCallback, this, _1));
  // Publish noisy odometry — consumed by kalman_filter and ekf_node
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/bumperbot_controller/odom_noisy", 10);

  // Pre-fill invariant odometry fields
  odom_msg_.header.frame_id = "odom";
  // child frame is base_footprint_ekf so the EKF can distinguish its input
  // from the clean odometry on odom → base_footprint
  odom_msg_.child_frame_id = "base_footprint_ekf";
  odom_msg_.pose.pose.orientation.x = 0.0;
  odom_msg_.pose.pose.orientation.y = 0.0;
  odom_msg_.pose.pose.orientation.z = 0.0;
  odom_msg_.pose.pose.orientation.w = 1.0; // identity

  // TF broadcaster — separate child frame to avoid conflicting with clean odom
  // TF
  transform_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  transform_stamped_.header.frame_id = "odom";
  transform_stamped_.child_frame_id = "base_footprint_noisy";

  prev_time_ = get_clock()->now();
}

void NoisyController::jointCallback(const sensor_msgs::msg::JointState &state) {
  // --- Noise injection ---
  // Seed the random engine with the current wall-clock nanoseconds so each
  // invocation gets a different seed and the noise isn't correlated.
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine noise_generator(seed);
  // Independent Gaussian distributions for each encoder (σ = 0.005 rad)
  std::normal_distribution<double> left_encoder_noise(0.0, 0.005);
  std::normal_distribution<double> right_encoder_noise(0.0, 0.005);

  // Corrupt the raw encoder readings before computing deltas
  double wheel_encoder_left =
      state.position.at(0) + left_encoder_noise(noise_generator);
  double wheel_encoder_right =
      state.position.at(1) + right_encoder_noise(noise_generator);

  // Angular displacement deltas using the noise-corrupted readings
  double dp_left = wheel_encoder_left - left_wheel_prev_pos_;
  double dp_right = wheel_encoder_right - right_wheel_prev_pos_;
  rclcpp::Time msg_time = state.header.stamp;
  rclcpp::Duration dt = msg_time - prev_time_;

  // Store RAW (non-noisy) values as the baseline for the next delta so
  // cumulative noise errors don't compound across callbacks
  left_wheel_prev_pos_ = state.position.at(0);
  right_wheel_prev_pos_ = state.position.at(1);
  prev_time_ = state.header.stamp;

  // Angular velocity of each wheel (rad/s) — noisy
  double fi_left = dp_left / dt.seconds();
  double fi_right = dp_right / dt.seconds();

  // Differential-drive kinematics → noisy body velocities
  double linear = (wheel_radius_ * fi_right + wheel_radius_ * fi_left) / 2;
  double angular =
      (wheel_radius_ * fi_right - wheel_radius_ * fi_left) / wheel_separation_;

  // Integrate noisy wheel displacements into noisy pose estimate
  double d_s = (wheel_radius_ * dp_right + wheel_radius_ * dp_left) / 2;
  double d_theta =
      (wheel_radius_ * dp_right - wheel_radius_ * dp_left) / wheel_separation_;
  theta_ += d_theta;
  x_ += d_s * cos(theta_);
  y_ += d_s * sin(theta_);

  // Publish noisy odometry
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  odom_msg_.header.stamp = get_clock()->now();
  odom_msg_.pose.pose.position.x = x_;
  odom_msg_.pose.pose.position.y = y_;
  odom_msg_.pose.pose.orientation.x = q.getX();
  odom_msg_.pose.pose.orientation.y = q.getY();
  odom_msg_.pose.pose.orientation.z = q.getZ();
  odom_msg_.pose.pose.orientation.w = q.getW();
  odom_msg_.twist.twist.linear.x = linear;
  odom_msg_.twist.twist.angular.z = angular;
  odom_pub_->publish(odom_msg_);

  // Broadcast TF: odom → base_footprint_noisy
  transform_stamped_.transform.translation.x = x_;
  transform_stamped_.transform.translation.y = y_;
  transform_stamped_.transform.rotation.x = q.getX();
  transform_stamped_.transform.rotation.y = q.getY();
  transform_stamped_.transform.rotation.z = q.getZ();
  transform_stamped_.transform.rotation.w = q.getW();
  transform_stamped_.header.stamp = get_clock()->now();
  transform_broadcaster_->sendTransform(transform_stamped_);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NoisyController>("noisy_controller");
  rclcpp::spin(node); // block until SIGINT
  rclcpp::shutdown();
  return 0;
}