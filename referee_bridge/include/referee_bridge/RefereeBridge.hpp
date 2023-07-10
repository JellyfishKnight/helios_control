#ifndef REFEREEE_BRIDGE
#define REFEREEE_BRIDGE

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_control.h"



#include <Referee.h>
#include <CRC.h>

#include <rm_interfaces/msg/game_robot_hp.hpp>
#include <rm_interfaces/msg/power_heat_data.hpp>
#include <rm_interfaces/msg/shoot_data.hpp>

namespace helios_control {

class RefereeBridge : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(RefereeBridge);

    REFEREE_PUBLIC
    hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

    REFEREE_PUBLIC
    hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;

    REFEREE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    REFEREE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    REFEREE_PUBLIC
    hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

    REFEREE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

    REFEREE_PUBLIC
    hardware_interface::return_type read(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    REFEREE_PUBLIC
    hardware_interface::return_type write(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    RefereeBridge(const rclcpp::NodeOptions& options);

    ~RefereeBridge() = default;

private: 
    std::unique_ptr<serial::Serial> serial_port_;
    std::string serial_port_name_;
    int serial_port_baudrate_;
    int serial_port_timeout_;

    std::unique_ptr<FrameBuffer> header_receive_buffer_;

    int SOF_ = 0xA5;
    int TOF_ = 0xA6;

};

} // namespace helios_control

#endif