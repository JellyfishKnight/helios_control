#include "GM6020Hardware.hpp"
#include <hardware_interface/actuator_interface.hpp>
#include <rclcpp/logging.hpp>
#include <serial/serial.h>

namespace helios_control {
    hardware_interface::CallbackReturn GM6020Hardware::on_init(const hardware_interface::HardwareInfo & info) {
        // init info
        if (hardware_interface::ActuatorInterface::on_init(info) !=hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        // resize states and commands
        hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        // check the number of joints
        if (info.joints.size() > 4) {
            RCLCPP_ERROR(logger_, "The number of actuators should be less than 4");
            return hardware_interface::CallbackReturn::FAILURE;
        }
        // check the number of parameters
        if (info.hardware_parameters.size() != 1) {
            RCLCPP_ERROR(logger_, "need the name of serial");
        }
        // create serial
        serial_ = std::make_shared<serial::Serial>();
        if (!serial_) {
            RCLCPP_ERROR(logger_, "Unable to create a serial");
            return hardware_interface::CallbackReturn::ERROR;
        }
        const hardware_interface::ComponentInfo & joint = info_.joints[0];
        // RRBotModularJoint has exactly one state and command interface on each joint
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
            rclcpp::get_logger("RRBotModularJoint"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
            rclcpp::get_logger("RRBotModularJoint"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
            rclcpp::get_logger("RRBotModularJoint"), "Joint '%s' has %zu state interface. 1 expected.",
            joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
            rclcpp::get_logger("RRBotModularJoint"), "Joint '%s' have %s state interface. '%s' expected.",
            joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn GM6020Hardware::on_configure(const rclcpp_lifecycle::State & previous_state) {
        try {
        serial_->setPort(info_.hardware_parameters["serial_name"]);
        serial_->setBaudrate(115200);
        serial_->setFlowcontrol(serial::flowcontrol_none);
        serial_->setParity(serial::parity_none); // default is parity_none
        serial_->setStopbits(serial::stopbits_one);
        serial_->setBytesize(serial::eightbits);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_->setTimeout(timeout);
        } catch (serial::SerialException& e) {
            RCLCPP_ERROR(logger_, "throwed an exception while declare a serial : %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn GM6020Hardware::on_activate(const rclcpp_lifecycle::State & previous_state) {
        serial_->open();
        if (!serial_->isOpen()) {
            RCLCPP_ERROR(logger_, "Unable to open serial");
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn GM6020Hardware::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
        serial_->close();
        if (serial_->isOpen()) {
            RCLCPP_ERROR(logger_, "Unable to close serial");
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type GM6020Hardware::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces) {
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type GM6020Hardware::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces) {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn GM6020Hardware::on_cleanup(const rclcpp_lifecycle::State & previous_state) {
        serial_.reset();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type GM6020Hardware::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type GM6020Hardware::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
        for (auto )
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> GM6020Hardware::export_state_interfaces() {

    }

    std::vector<hardware_interface::CommandInterface> GM6020Hardware::export_command_interfaces() {
        
    }

    hardware_interface::CallbackReturn GM6020Hardware::on_error(const rclcpp_lifecycle::State & previous_state) {
        
    }

}