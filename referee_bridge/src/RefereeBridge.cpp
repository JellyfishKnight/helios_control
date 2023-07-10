#include "RefereeBridge.hpp"

namespace helios_control {
    hardware_interface::CallbackReturn RefereeBridge::on_init(const hardware_interface::HardwareInfo & info) {
        RCLCPP_INFO(logger_, "on_init");
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        hardware_info_ = info;
        serial_port_ = std::make_unique<serial::Serial>();
        header_receive_buffer_ = std::make_unique<FrameBuffer>();
        serial_port_name_ = SERIAL_PORT_NAME;
        serial_port_baudrate_ = SERIAL_PORT_BAUDRATE;
        serial_port_timeout_ = SERIAL_PORT_TIMEOUT;
        if (serial_port_ == nullptr || header_receive_buffer_ == nullptr) {
            RCLCPP_ERROR(logger_, "Unable to allocate memory for serial port or frame buffer");
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RefereeBridge::on_configure(const rclcpp_lifecycle::State & previous_state) {
        RCLCPP_INFO(logger_, "on_configure");
        try {
            RCLCPP_INFO(logger_, "Openning serial port %s", serial_port_name_.c_str());
            serial_port_->setPort(serial_port_name_);
            serial_port_->setBaudrate(serial_port_baudrate_);
            serial_port_->setFlowcontrol(serial::flowcontrol_none);
            serial_port_->setParity(serial::parity_none); // default is parity_none
            serial_port_->setStopbits(serial::stopbits_one);
            serial_port_->setBytesize(serial::eightbits);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(serial_port_timeout_);
            serial_port_->setTimeout(timeout);
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(logger_, "Unable to initialize port: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
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
        try {
            serial_port_->open();
        } catch(serial::IOException& e) {
            RCLCPP_ERROR(logger_, "Unable to open port: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (serial_port_->isOpen()) {
            RCLCPP_INFO(logger_, "Serial Port Activated");
        } else {
            RCLCPP_ERROR(logger_, "Serial Port Failed Activating");
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RefereeBridge::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
        RCLCPP_INFO(rclcpp::get_logger("RefereeBridge"), "on_deactivate");
        try {
            serial_port_->close();
        } catch(serial::IOException& e) {
            RCLCPP_ERROR(logger_, "Unable to close port: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (!serial_port_->isOpen()) {
            RCLCPP_INFO(logger_, "Serial Port Deactivated");
        } else {
            RCLCPP_ERROR(logger_, "Serial Port Failed Deactivating");
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RefereeBridge::on_cleanup(const rclcpp_lifecycle::State & previous_state) {
        RCLCPP_INFO(rclcpp::get_logger("RefereeBridge"), "on_cleanup");
        serial_port_.reset();
        header_receive_buffer_.reset();
        if (serial_port_ != nullptr || header_receive_buffer_ != nullptr) {
            RCLCPP_ERROR(logger_, "Unable to deallocate memory for serial port or frame buffer");
            return hardware_interface::CallbackReturn::ERROR;
        }
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
