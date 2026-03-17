// =============================================================================
// bumperbot_interface.hpp  —  bumperbot_firmware
// =============================================================================
// Header for the ros2_control SystemInterface plugin that bridges ROS 2 and
// the Arduino motor controller over a serial (UART) connection.
//
// This class is loaded as a pluginlib plugin by controller_manager at startup
// (declared in bumperbot_interface.xml). It is selected when the URDF is
// processed with is_sim:=False in hardware_interface.launch.py.
//
// Key lifecycle methods (called by controller_manager):
//   on_init()       — read serial port name from URDF hardware_parameters
//   on_activate()   — open serial port at 115200 baud
//   on_deactivate() — close serial port
//   read()          — parse encoder feedback string from Arduino → state
//   interfaces write()         — format velocity commands → serial string →
//   Arduino
//
// State interfaces exported:   position + velocity for each wheel joint
// Command interfaces exported: velocity for each wheel joint
// =============================================================================

#ifndef BUMPERBOT_INTERFACE_HPP
#define BUMPERBOT_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp> // base class for system HW interfaces
#include <libserial/SerialPort.h> // C++ serial port abstraction
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <string>
#include <vector>

namespace bumperbot_firmware {

// Convenience alias so we don't repeat the full lifecycle return type
// everywhere
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class BumperbotInterface : public hardware_interface::SystemInterface {
public:
  BumperbotInterface();
  virtual ~BumperbotInterface();

  // --- Lifecycle callbacks (called by controller_manager) ---

  // Called when the controller_manager activates the hardware plugin.
  // Opens the serial port and initialises state/command buffers.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  // Called when the controller_manager deactivates the hardware plugin.
  // Closes the serial port cleanly.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // --- SystemInterface overrides ---

  // Reads the serial port name from URDF <hardware><param name="port">
  CallbackReturn
  on_init(const hardware_interface::HardwareInfo &hardware_info) override;

  // Registers position and velocity state interfaces for each wheel joint
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  // Registers velocity command interfaces for each wheel joint
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  // Called every control loop — reads serial feedback and updates state
  // interfaces
  hardware_interface::return_type read(const rclcpp::Time &,
                                       const rclcpp::Duration &) override;

  // Called every control loop — formats and sends velocity commands over serial
  hardware_interface::return_type write(const rclcpp::Time &,
                                        const rclcpp::Duration &) override;

private:
  LibSerial::SerialPort arduino_; // handle to the open serial port
  std::string port_;              // port path, e.g. "/dev/ttyUSB0"

  // Buffers shared between the hardware interface and controller_manager.
  // controller_manager writes into velocity_commands_ (write path) and
  // reads from position_states_ / velocity_states_ (read path).
  std::vector<double> velocity_commands_; // rad/s target per wheel
  std::vector<double> position_states_;   // accumulated angle per wheel (rad)
  std::vector<double> velocity_states_;   // current speed per wheel (rad/s)

  rclcpp::Time
      last_run_; // timestamp of the last successful serial read (for dt)
};
} // namespace bumperbot_firmware

#endif // BUMPERBOT_INTERFACE_HPP