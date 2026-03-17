// =============================================================================
// imu_republisher.cpp  —  bumperbot_localization
// =============================================================================
// Thin relay node that bridges the raw IMU topic into the EKF pipeline.
// C++ default version — used when use_python:=False in
// local_localization.launch.py. Python equivalent:
// bumperbot_localization/imu_republisher.py
//
// Problem: robot_localization's ekf_node requires all sensor inputs to use the
// EKF base frame ("base_footprint_ekf") as their header.frame_id, but the raw
// /imu/out topic from mpu6050_driver.py uses "base_footprint".
//
// Solution: subscribe to /imu/out, copy the message, replace the frame_id,
// and republish on /imu_ekf — which is the topic ekf.yaml points to.
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Allows "1s" duration literal in rclcpp::sleep_for
using namespace std::chrono_literals;

// Global publisher declared outside main() so the bare-function callback
// (imuCallback) can access it without needing a class self reference
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

void imuCallback(const sensor_msgs::msg::Imu &imu) {
  // Copy the full message so we don't mutate the incoming shared_ptr
  sensor_msgs::msg::Imu new_imu;
  new_imu = imu;
  // Swap the frame_id to the EKF base frame — only this field changes
  new_imu.header.frame_id = "base_footprint_ekf";
  imu_pub->publish(new_imu);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("imu_republisher_node");

  // Brief sleep to ensure the IMU driver (and other upstream nodes) have
  // started publishing before we begin subscribing and forwarding
  rclcpp::sleep_for(1s);

  // Publish on /imu_ekf — the topic name expected by ekf.yaml's imu0 config
  imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu_ekf", 10);
  // Subscribe to the raw IMU topic from mpu6050_driver (or the Gazebo bridge)
  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>("imu/out", 10,
                                                                  imuCallback);

  rclcpp::spin(node);
  rclcpp::shutdown();
}