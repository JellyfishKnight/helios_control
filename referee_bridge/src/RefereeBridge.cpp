#include "RefereeBridge.hpp"

namespace helios_control {
    hardware_interface::CallbackReturn RefereeBridge::on_init(const hardware_interface::HardwareInfo & info) {
        RCLCPP_INFO(rclcpp::get_logger("RefereeBridge"), "on_init");
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        serial_port_ = std::make_unique<serial::Serial>();
        header_receive_buffer_ = std::make_unique<FrameBuffer>();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RefereeBridge::on_configure(const rclcpp_lifecycle::State & previous_state) {
        RCLCPP_INFO(rclcpp::get_logger("RefereeBridge"), "on_configure");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RefereeBridge::export_state_interfaces() {
        RCLCPP_INFO(rclcpp::get_logger("RefereeBridge"), "export_state_interfaces");
        return std::vector<hardware_interface::StateInterface>();
    }

    std::vector<hardware_interface::CommandInterface> RefereeBridge::export_command_interfaces() {
        RCLCPP_INFO(rclcpp::get_logger("RefereeBridge"), "export_command_interfaces");
        return std::vector<hardware_interface::CommandInterface>();
    }

    hardware_interface::CallbackReturn RefereeBridge::on_activate(const rclcpp_lifecycle::State & previous_state) {
        RCLCPP_INFO(rclcpp::get_logger("RefereeBridge"), "on_activate");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RefereeBridge::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
        RCLCPP_INFO(rclcpp::get_logger("RefereeBridge"), "on_deactivate");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RefereeBridge::read(const rclcpp::Time & , const rclcpp::Duration & )
    {

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RefereeBridge::write(const rclcpp::Time & , const rclcpp::Duration & )
    {

        return hardware_interface::return_type::OK;
    }

} // namespace helios_control
