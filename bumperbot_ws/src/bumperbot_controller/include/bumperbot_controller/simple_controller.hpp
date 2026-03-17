// =============================================================================
// simple_controller.hpp  —  bumperbot_controller
// =============================================================================
// Header for the SimpleController ROS 2 node (C++ version).
// Simulation / tutorial only — NOT used on the real robot.
// The real robot uses NoisyController (noisy_controller.hpp/.cpp) instead.
//
// SimpleController implements clean differential-drive kinematics:
//   1. velCallback  — forward kinematics: desired (v, w) → wheel speeds
//      Publishes Float64MultiArray on /simple_velocity_controller/commands
//   2. jointCallback — inverse kinematics: wheel encoder deltas → (x, y, θ)
//      Publishes Odometry on /bumperbot_controller/odom
//      Broadcasts TF: odom → base_footprint
//
// Uses Eigen for the 2×2 kinematic matrix inversion.
// =============================================================================

#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <Eigen/Core> // 2×2 speed conversion matrix
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp> // desired robot velocity (v, w)
#include <nav_msgs/msg/odometry.hpp>           // computed pose + velocity
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>      // wheel encoder positions
#include <std_msgs/msg/float64_multi_array.hpp> // per-wheel speed commands
#include <tf2_ros/transform_broadcaster.h> // broadcasts odom → base_footprint TF

class SimpleController : public rclcpp::Node {
public:
  // Constructs the node with the given name; sets up pubs, subs, and the
  // kinematic conversion matrix from wheel geometry parameters.
  SimpleController(const std::string &name);

private:
  // Forward kinematics: converts cmd_vel (v, w) → per-wheel angular speeds
  void velCallback(const geometry_msgs::msg::TwistStamped &msg);

  // Inverse kinematics: integrates encoder deltas into a pose estimate
  void jointCallback(const sensor_msgs::msg::JointState &msg);

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      vel_sub_; // /bumperbot_controller/cmd_vel
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      wheel_cmd_pub_; // /simple_velocity_controller/commands
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_sub_; // /joint_states
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      odom_pub_; // /bumperbot_controller/odom

  // --- Wheel geometry & kinematic matrix ---
  double wheel_radius_;     // metres (ROS param: wheel_radius)
  double wheel_separation_; // metres (ROS param: wheel_separation)
  // 2×2 matrix M mapping [v, w] → [ω_right, ω_left] in the diff-drive model
  Eigen::Matrix2d speed_conversion_;

  // --- Odometry state ---
  double right_wheel_prev_pos_; // previous right wheel angle (rad)
  double left_wheel_prev_pos_;  // previous left wheel angle (rad)
  rclcpp::Time prev_time_;      // timestamp of the last joint callback
  nav_msgs::msg::Odometry
      odom_msg_; // reused message to avoid per-cycle allocation
  double x_;     // accumulated x position in odom frame (m)
  double y_;     // accumulated y position in odom frame (m)
  double theta_; // accumulated heading in odom frame (rad)

  // --- TF broadcaster ---
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  geometry_msgs::msg::TransformStamped
      transform_stamped_; // pre-filled, stamped per cycle
};

#endif // SIMPLE_CONTROLLER_HPP