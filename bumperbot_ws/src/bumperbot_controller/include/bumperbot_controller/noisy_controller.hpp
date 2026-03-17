// =============================================================================
// noisy_controller.hpp  —  bumperbot_controller
// =============================================================================
// Header for the NoisyController ROS 2 node (C++ version).
// This is the controller used on the REAL ROBOT (selected by
// real_robot.launch.py with use_simple_controller:=False, use_python:=False).
//
// Identical in structure to SimpleController but does NOT process cmd_vel /
// publish wheel speed commands — it only estimates odometry.
// The bumperbot_controller (ros2_control velocity PID) handles actual wheel
// speed control; NoisyController only reads encoder feedback and estimates
// pose.
//
// Key difference from SimpleController:
//   - Adds Gaussian noise (σ=0.005 rad, using std::normal_distribution) to
//     wheel encoder readings before computing odometry.
//   - Publishes on /bumperbot_controller/odom_noisy (not /odom) and broadcasts
//     odom → base_footprint_noisy TF so both estimates coexist.
// =============================================================================

#ifndef NOISY_CONTROLLER_HPP
#define NOISY_CONTROLLER_HPP

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp> // noisy odometry output
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp> // wheel encoder positions from hardware interface
#include <tf2_ros/transform_broadcaster.h> // broadcasts odom → base_footprint_noisy TF

class NoisyController : public rclcpp::Node {
public:
  // Constructs the node; subscribes to /joint_states, publishes odom_noisy
  NoisyController(const std::string &name);

private:
  // Inverse kinematics with noise injection: encoder deltas + Gaussian noise →
  // pose
  void jointCallback(const sensor_msgs::msg::JointState &msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_sub_; // /joint_states
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      odom_pub_; // /bumperbot_controller/odom_noisy

  // --- Wheel geometry ---
  double wheel_radius_;     // metres (ROS param, intentionally offset by
                            // wheel_radius_error)
  double wheel_separation_; // metres (ROS param, intentionally offset by
                            // wheel_separation_error)

  // --- Odometry state ---
  double
      right_wheel_prev_pos_; // previous right wheel angle for delta computation
  double
      left_wheel_prev_pos_; // previous left wheel angle for delta computation
  rclcpp::Time prev_time_;
  nav_msgs::msg::Odometry odom_msg_; // pre-allocated, reused each callback
  double x_;                         // accumulated x in odom frame (noisy)
  double y_;                         // accumulated y in odom frame (noisy)
  double theta_; // accumulated heading in odom frame (noisy)

  // --- TF broadcaster ---
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_stamped_;
};

#endif // NOISY_CONTROLLER_HPP