#include "RefereeBridge.hpp"


RefereeBridge::RefereeBridge(const rclcpp::NodeOptions& options) : Node("referee_bridge", options) {
    DelcareParams();
    RegisterTopics();
    InitSerial();
    std::thread([this]() {
        RCLCPP_INFO(this->get_logger(),"thread start");
        this->ProcessFuntion(); 
    }).detach();
}

void RefereeBridge::RegisterTopics() {
    game_robot_hp_pub_ = this->create_publisher<rm_interfaces::msg::GameRobotHP>("game_robot_hp", rclcpp::SensorDataQoS());
    power_heat_data_pub_ = this->create_publisher<rm_interfaces::msg::PowerHeatData>("power_heat_data", rclcpp::SensorDataQoS());
    shoot_data_pub_ = this->create_publisher<rm_interfaces::msg::ShootData>("shoot_data", rclcpp::SensorDataQoS());
}

void RefereeBridge::InitSerial() {
    serial_port_name_ = this->get_parameter("serial_port_name").as_string();
    serial_port_baudrate_ = this->get_parameter("serial_port_baudrate").as_int();
    serial_port_timeout_ = this->get_parameter("serial_port_timeout").as_int();
    try {
        RCLCPP_INFO(this->get_logger(), "serial_port:[%s]", serial_port_name_.c_str());
        serial_port_.setPort(serial_port_name_);
        serial_port_.setBaudrate(serial_port_baudrate_);
        serial_port_.setFlowcontrol(serial::flowcontrol_none);
        serial_port_.setParity(serial::parity_none); // default is parity_none
        serial_port_.setStopbits(serial::stopbits_one);
        serial_port_.setBytesize(serial::eightbits);
        serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_port_timeout_);
        serial_port_.setTimeout(time_out);
        serial_port_.open();
    } catch (serial::IOException &e) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
        exit(0);
    }
    if (serial_port_.isOpen()) {
        RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unable to initial Serial port ");
        exit(0);
    }
}

void RefereeBridge::DelcareParams() {
    this->declare_parameter("serial_port_name", "/dev/ttyUSB1");
    this->declare_parameter("serial_port_baudrate", 115200);
    this->declare_parameter("serial_port_timeout", 1000);
}

void RefereeBridge::ProcessFuntion() {
    while (rclcpp::ok()) {
        if (!serial_port_.isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "serial port is not open");
            rclcpp::shutdown();
            exit(0);
        }
        // check head start   检查起始 数据帧头
        uint8_t pdata[16] = {0x00};
        if (serial_port_.read(pdata, 1) == 1 && pdata[0] == 0xa5)
        {
            try {
                serial_port_.read((uint8_t *)&header_receive_buffer_.data_length, 6);
                if (header_receive_buffer_.data_length > 256)
                {
                    RCLCPP_ERROR(this->get_logger(), "data length error");
                    continue;
                }
                serial_port_.read(header_receive_buffer_.data, header_receive_buffer_.data_length + 2);
                if (header_receive_buffer_.CheckTail() != 0xa6a7 && !header_receive_buffer_.CheckPackCRC16())
                {
                    RCLCPP_WARN(this->get_logger(), "check cmd id %x tail failed %x", header_receive_buffer_.cmd_id, header_receive_buffer_.CheckTail());
                    continue;
                }
            } catch (serial::SerialException& exception) {
                RCLCPP_ERROR(this->get_logger(), "serial port read error: %s", exception.what());
                continue;
            }
            
            
        }
}

