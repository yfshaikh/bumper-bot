// =============================================================================
// simple_controller.cpp  —  bumperbot_controller
// =============================================================================
// C++ implementation of SimpleController — a clean differential-drive
// odometry node used in simulation / tutorial mode.
// NOT used on the real robot. Selected when use_simple_controller:=True.
//
// See simple_controller.hpp for the full class description.
// The Python equivalent is bumperbot_controller/simple_controller.py.
// =============================================================================

#include "bumperbot_controller/simple_controller.hpp"
#include <Eigen/Geometry>              // needed for Eigen matrix inverse
#include <tf2/LinearMath/Quaternion.h> // tf2::Quaternion for yaw → quaternion

// std::placeholders::_1 used in std::bind for subscription callbacks
using std::placeholders::_1;

SimpleController::SimpleController(const std::string &name)
    // Initialise base Node with the given name
    : Node(name)
      // Zero out odometry state on construction
      ,
      left_wheel_prev_pos_(0.0), right_wheel_prev_pos_(0.0), x_(0.0), y_(0.0),
      theta_(0.0) {
  // Declare ROS parameters so they can be overridden from the launch file
  declare_parameter("wheel_radius", 0.033);    // metres
  declare_parameter("wheel_separation", 0.17); // metres
  wheel_radius_ = get_parameter("wheel_radius").as_double();
  wheel_separation_ = get_parameter("wheel_separation").as_double();
  RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius " << wheel_radius_);
  RCLCPP_INFO_STREAM(get_logger(),
                     "Using wheel separation " << wheel_separation_);

  // Publisher: per-wheel speed targets for simple_velocity_controller
  wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/simple_velocity_controller/commands", 10);
  // Subscriber: desired robot velocity from joy_teleop / nav stack
  vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/bumperbot_controller/cmd_vel", 10,
      std::bind(&SimpleController::velCallback, this, _1));
  // Subscriber: wheel encoder positions from joint_state_broadcaster
  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&SimpleController::jointCallback, this, _1));
  // Publisher: clean odometry estimate
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/bumperbot_controller/odom", 10);

  // Build the 2×2 speed conversion matrix M from wheel geometry.
  // M maps [v (m/s), w (rad/s)] → [ω_right (rad/s), ω_left (rad/s)].
  // Derived from the differential drive equations:
  //   v = r*(ω_r+ω_l)/2  →  [v, w] = M * [ω_r, ω_l]
  // Using comma-initialiser syntax from Eigen:
  speed_conversion_ << wheel_radius_ / 2, wheel_radius_ / 2,
      wheel_radius_ / wheel_separation_, -wheel_radius_ / wheel_separation_;
  RCLCPP_INFO_STREAM(get_logger(), "The conversion matrix is \n"
                                       << speed_conversion_);

  // Pre-fill invariant fields so they're not set on every callback
  odom_msg_.header.frame_id = "odom";
  odom_msg_.child_frame_id = "base_footprint";
  odom_msg_.pose.pose.orientation.x = 0.0;
  odom_msg_.pose.pose.orientation.y = 0.0;
  odom_msg_.pose.pose.orientation.z = 0.0;
  odom_msg_.pose.pose.orientation.w = 1.0; // identity quaternion (no rotation)

  // Set up the TF broadcaster and pre-fill the invariant frame IDs
  transform_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  transform_stamped_.header.frame_id = "odom";
  transform_stamped_.child_frame_id = "base_footprint";

  prev_time_ = get_clock()->now();
}

void SimpleController::velCallback(
    const geometry_msgs::msg::TwistStamped &msg) {
  // Forward kinematics: given desired robot velocity (v, w), compute
  // the required angular speed for each wheel.
  // wheel_speed = M⁻¹ * [v, w]ᵀ
  Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z);
  // Eigen's .inverse() computes M⁻¹ efficiently for a 2×2 matrix
  Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;

  std_msgs::msg::Float64MultiArray wheel_speed_msg;
  // Convention expected by simple_velocity_controller: [right, left]
  wheel_speed_msg.data.push_back(wheel_speed.coeff(1)); // right wheel
  wheel_speed_msg.data.push_back(wheel_speed.coeff(0)); // left wheel

  wheel_cmd_pub_->publish(wheel_speed_msg);
}

void SimpleController::jointCallback(
    const sensor_msgs::msg::JointState &state) {
  // Inverse kinematics: given encoder delta positions, compute (x, y, θ).

  // Angular displacement of each wheel since the last callback (radians)
  double dp_left = state.position.at(0) - left_wheel_prev_pos_;
  double dp_right = state.position.at(1) - right_wheel_prev_pos_;
  // Elapsed time — use the message timestamp for deterministic integration
  rclcpp::Time msg_time = state.header.stamp;
  rclcpp::Duration dt = msg_time - prev_time_;

  // Store current positions as baseline for the next delta
  left_wheel_prev_pos_ = state.position.at(0);
  right_wheel_prev_pos_ = state.position.at(1);
  prev_time_ = state.header.stamp;

  // Compute angular velocity of each wheel (rad/s) via finite difference
  double fi_left = dp_left / dt.seconds();
  double fi_right = dp_right / dt.seconds();

  // Differential-drive kinematic model → robot body velocities
  double linear =
      (wheel_radius_ * fi_right + wheel_radius_ * fi_left) / 2; // m/s
  double angular = (wheel_radius_ * fi_right - wheel_radius_ * fi_left) /
                   wheel_separation_; // rad/s

  // Integrate wheel displacements into pose (arc-length approximation)
  double d_s =
      (wheel_radius_ * dp_right + wheel_radius_ * dp_left) / 2; // arc length
  double d_theta =
      (wheel_radius_ * dp_right - wheel_radius_ * dp_left) / wheel_separation_;
  theta_ += d_theta;
  x_ += d_s * cos(theta_); // project arc along current heading
  y_ += d_s * sin(theta_);

  // Compose and publish odometry message
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_); // roll=0, pitch=0, yaw=theta_ → quaternion
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

  // Broadcast TF: odom → base_footprint so rviz / nav stack know robot pose
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
  auto node = std::make_shared<SimpleController>("simple_controller");
  rclcpp::spin(node); // block until SIGINT
  rclcpp::shutdown();
  return 0;
}