// =============================================================================
// bumperbot_interface.cpp  —  bumperbot_firmware
// =============================================================================
// Implementation of the ros2_control SystemInterface plugin for communicating
// with the Arduino motor controller over serial (UART).
//
// Serial protocol (both directions use the same format):
//   Command → Arduino:  "r<sign><value>,l<sign><value>,"
//                        e.g. "rp12.34,ln05.67,"   (right positive, left
//                        negative)
//   Feedback ← Arduino: "r<sign><value>,l<sign><value>"
//                        e.g. "rp11.20,lp04.90"
//
// Where 'p' = positive direction, 'n' = negative direction.
//
// This file is loaded as a shared library by controller_manager at runtime
// via the pluginlib export declared at the bottom of this file.
// =============================================================================

#include "bumperbot_firmware/bumperbot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp> // HW_IF_POSITION etc.
#include <pluginlib/class_list_macros.hpp> // PLUGINLIB_EXPORT_CLASS macro

namespace bumperbot_firmware {
BumperbotInterface::BumperbotInterface() {}

BumperbotInterface::~BumperbotInterface() {
  // Ensure the serial port is closed when the plugin is unloaded.
  // Guard with IsOpen() to avoid closing an already-closed port.
  if (arduino_.IsOpen()) {
    try {
      arduino_.Close();
    } catch (...) {
      RCLCPP_FATAL_STREAM(
          rclcpp::get_logger("BumperbotInterface"),
          "Something went wrong while closing connection with port " << port_);
    }
  }
}

CallbackReturn BumperbotInterface::on_init(
    const hardware_interface::HardwareInfo &hardware_info) {
  // Call the base class on_init first — it parses the URDF <ros2_control> block
  // and populates info_.joints, info_.hardware_parameters, etc.
  CallbackReturn result =
      hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  // Read the serial port path from the URDF <hardware> parameter block.
  // Example URDF: <param name="port">/dev/ttyUSB0</param>
  try {
    port_ = info_.hardware_parameters.at("port");
  } catch (const std::out_of_range &e) {
    RCLCPP_FATAL(rclcpp::get_logger("BumperbotInterface"),
                 "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  // Pre-allocate buffers sized to the number of joints in the URDF
  // (typically 2: wheel_right_joint, wheel_left_joint)
  velocity_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  velocity_states_.reserve(info_.joints.size());
  last_run_ = rclcpp::Clock().now();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
BumperbotInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Register both position and velocity state interfaces for every joint.
  // controller_manager reads these to publish /joint_states.
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &velocity_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
BumperbotInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Register a velocity command interface for every joint.
  // The velocity controller writes target rad/s values into these interfaces,
  // which write() then serialises and sends to the Arduino.
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &velocity_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn
BumperbotInterface::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"),
              "Starting robot hardware ...");

  // Zero-out all command and state buffers before opening the serial port
  velocity_commands_ = {0.0, 0.0};
  position_states_ = {0.0, 0.0};
  velocity_states_ = {0.0, 0.0};

  try {
    // Open the serial port and set it to 115200 baud — must match the
    // Arduino firmware Serial.begin(115200) in robot_control.ino
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  } catch (...) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"),
                        "Something went wrong while interacting with port "
                            << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
BumperbotInterface::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"),
              "Stopping robot hardware ...");

  if (arduino_.IsOpen()) {
    try {
      arduino_.Close();
    } catch (...) {
      RCLCPP_FATAL_STREAM(
          rclcpp::get_logger("BumperbotInterface"),
          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
BumperbotInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
  // Only attempt to parse when there is a complete line waiting in the buffer
  if (arduino_.IsDataAvailable()) {
    // dt since the last read — used to integrate velocity into position
    auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    std::string message;
    arduino_.ReadLine(message); // e.g. "rp12.34,lp05.67,\n"

    // Split the comma-separated message into tokens and decode each field
    std::stringstream ss(message);
    std::string res;
    int multiplier = 1;
    while (std::getline(ss, res, ',')) {
      // res.at(1) is the sign character: 'p' → positive, 'n' → negative
      multiplier = res.at(1) == 'p' ? 1 : -1;

      if (res.at(0) == 'r') // right wheel token, e.g. "rp12.34"
      {
        // Parse the numeric velocity and apply the direction sign
        velocity_states_.at(0) =
            multiplier * std::stod(res.substr(2, res.size()));
        // Integrate velocity × dt to accumulate wheel position (rad)
        position_states_.at(0) += velocity_states_.at(0) * dt;
      } else if (res.at(0) == 'l') // left wheel token, e.g. "lp05.67"
      {
        velocity_states_.at(1) =
            multiplier * std::stod(res.substr(2, res.size()));
        position_states_.at(1) += velocity_states_.at(1) * dt;
      }
    }
    last_run_ = rclcpp::Clock().now();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
BumperbotInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
  // Format velocity commands as the wire protocol expected by
  // robot_control.ino:
  //   "r<sign><zero-padded value>,l<sign><zero-padded value>,"
  //   e.g. "rp12.34,ln05.67,"
  std::stringstream message_stream;
  // Determine direction sign from the sign of the velocity command
  char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
  char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';

  // Zero-pad single-digit values so the Arduino parser always receives a fixed
  // 5-character numeric field (XX.XX), e.g. "05.67" not "5.67"
  std::string compensate_zeros_right = "";
  std::string compensate_zeros_left = "";
  if (std::abs(velocity_commands_.at(0)) < 10.0) {
    compensate_zeros_right = "0";
  } else {
    compensate_zeros_right = "";
  }
  if (std::abs(velocity_commands_.at(1)) < 10.0) {
    compensate_zeros_left = "0";
  } else {
    compensate_zeros_left = "";
  }

  // Assemble the final wire message with 2 decimal places
  message_stream << std::fixed << std::setprecision(2) << "r"
                 << right_wheel_sign << compensate_zeros_right
                 << std::abs(velocity_commands_.at(0)) << ",l"
                 << left_wheel_sign << compensate_zeros_left
                 << std::abs(velocity_commands_.at(1)) << ",";

  try {
    arduino_.Write(message_stream.str());
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("BumperbotInterface"),
                        "Something went wrong while sending the message "
                            << message_stream.str() << " to the port "
                            << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
} // namespace bumperbot_firmware

// Register the plugin with pluginlib so controller_manager can discover it
// by looking for hardware_interface::SystemInterface plugins.
PLUGINLIB_EXPORT_CLASS(bumperbot_firmware::BumperbotInterface,
                       hardware_interface::SystemInterface)