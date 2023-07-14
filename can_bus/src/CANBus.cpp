#include "CANBus.hpp"

namespace helios_control {
    hardware_interface::CallbackReturn CANBus::on_init(const hardware_interface::HardwareInfo & info) {

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CANBus::on_configure(const rclcpp_lifecycle::State & previous_state) {
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> CANBus::export_state_interfaces() {
            
    }

    std::vector<hardware_interface::CommandInterface> CANBus::export_command_interfaces() {

    }

    hardware_interface::CallbackReturn CANBus::on_activate(const rclcpp_lifecycle::State & previous_state) {

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CANBus::on_deactivate(const rclcpp_lifecycle::State & previous_state) {

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type CANBus::read(const rclcpp::Time & time, const rclcpp::Duration & period) {

        return hardware_interface::return_type::OK;
            
    }

    hardware_interface::return_type CANBus::write(const rclcpp::Time & time, const rclcpp::Duration & period) {

        return hardware_interface::return_type::OK;
    }

} // namespace helios_control