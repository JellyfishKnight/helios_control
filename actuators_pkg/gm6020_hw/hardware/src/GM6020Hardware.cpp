#include "GM6020Hardware.hpp"

namespace helios_control {
    hardware_interface::CallbackReturn GM6020Hardware::on_init(const hardware_interface::HardwareInfo & info) {
        if (hardware_interface::SystemInterface::on_init(info) !=hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    hardware_interface::CallbackReturn GM6020Hardware::on_configure(const rclcpp_lifecycle::State & previous_state) {

    }

    hardware_interface::CallbackReturn GM6020Hardware::on_activate(const rclcpp_lifecycle::State & previous_state) {

    }

    hardware_interface::CallbackReturn GM6020Hardware::on_deactivate(const rclcpp_lifecycle::State & previous_state) {

    }

    hardware_interface::return_type GM6020Hardware::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces) {
        
    }

    hardware_interface::return_type GM6020Hardware::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces) {
        
    }

    hardware_interface::CallbackReturn GM6020Hardware::on_cleanup(const rclcpp_lifecycle::State & previous_state) {

    }

    hardware_interface::return_type GM6020Hardware::read(const rclcpp::Time & time, const rclcpp::Duration & period) {

    }

    hardware_interface::return_type GM6020Hardware::write(const rclcpp::Time & time, const rclcpp::Duration & period) {

    }

    std::vector<hardware_interface::StateInterface> GM6020Hardware::export_state_interfaces() {

    }

    std::vector<hardware_interface::CommandInterface> GM6020Hardware::export_command_interfaces() {
        
    }

    hardware_interface::CallbackReturn GM6020Hardware::on_error(const rclcpp_lifecycle::State & previous_state) {
        
    }

}