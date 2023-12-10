#include "MotorPacket.hpp"
#include <cmath>
#include <cstdint>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <utility>

namespace math_utilities {

MotorPacket::MotorPacket(std::string motor_name) :
    motor_name_(std::move(motor_name))
{
    // init all member
    total_angle_ = 0;
    speed_rpm_ = 0;
    round_cnt_ = 0;
    temperature_ = 0;
    real_current_ = 0;
    given_current_ = 0;
    mid_angle_ = 0;
}

void MotorPacket::calculate_motor_measure(MotorPacket motor_packet) {
    total_angle_ = motor_packet.total_angle_;
    real_current_ = motor_packet.real_current_;
    given_current_ = motor_packet.given_current_;
    temperature_ = motor_packet.temperature_;
    speed_rpm_ = real_current_; //这里是因为两种电调对应位不一样的信息
}

///TODO: need improve: extra caculation
void MotorPacket::get_moto_measure(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces, std::map<std::string, MotorPacket>& motor_map) {
    // translate state_interfaces
    std::map<std::string, MotorPacket> temp_map;
    for (auto& state : state_interfaces) {
        if (temp_map.find(state.get_prefix_name()) == temp_map.end()) {
            MotorPacket temp_packet(state.get_prefix_name());
            temp_map.emplace(std::pair<std::string, MotorPacket>(state.get_prefix_name(), temp_packet));
        }
        auto packet = temp_map.find(state.get_prefix_name());
        if (state.get_interface_name() == TOTAL_ANGLE) {
            packet->second.total_angle_ = static_cast<int64_t>(state.get_value());
        } else if (state.get_interface_name() == VELOCITY) {
            packet->second.real_current_ = static_cast<int16_t>(state.get_value());
        } else if (state.get_interface_name() == CURRENT) {
            packet->second.given_current_ = static_cast<int16_t>(state.get_value());
        } else if (state.get_interface_name() == CAN_ID) {
            packet->second.can_id_ = static_cast<uint8_t>(state.get_value());
        } else if (state.get_interface_name() == MOTOR_TYPE) {
            packet->second.motor_type_ = static_cast<int>(state.get_value());
        } else if (state.get_interface_name() == MOTOR_ID) {
            packet->second.motor_id_ = static_cast<uint8_t>(state.get_value());
        } else if (state.get_interface_name() == TEMPERATURE) {
            packet->second.temperature_ = static_cast<uint8_t>(state.get_value());
        }
    }
    std::map<UniqueFlag, MotorPacket> unique_map;
    for (auto iter = temp_map.begin(); iter != temp_map.end(); iter++) {
        UniqueFlag temp_flag;
        temp_flag.can_id = iter->second.can_id_;
        temp_flag.motor_type = iter->second.motor_type_;
        temp_flag.motor_id = iter->second.motor_id_;
        unique_map.emplace(std::pair<UniqueFlag, MotorPacket>(temp_flag, iter->second));
    }
    for (auto& motor_packet : motor_map) {
        UniqueFlag temp_flag;
        temp_flag.can_id = motor_packet.second.can_id_;
        temp_flag.motor_type = motor_packet.second.motor_type_;
        temp_flag.motor_id = motor_packet.second.motor_id_;
        auto iter = unique_map.find(temp_flag);
        if (iter != unique_map.end()) {
            motor_packet.second.calculate_motor_measure(iter->second);
            RCLCPP_DEBUG(rclcpp::get_logger("MotorPacket"), "can_id: %d, motor_type: %d, motor_id: %d", temp_flag.can_id, temp_flag.motor_type, temp_flag.motor_id);
        } else {
            // RCLCPP_WARN(rclcpp::get_logger("MotorPacket"), "can_id: %d, motor_type: %d, motor_id: %d", temp_flag.can_id, temp_flag.motor_type, temp_flag.motor_id);
            // RCLCPP_WARN(rclcpp::get_logger("MotorPacket"), "can't find motor state: %s", motor_packet.first.c_str());
        }
    }
}

void MotorPacket::set_state_msg(helios_control_interfaces::msg::MotorState& motor_state) {
    motor_state.full_name = motor_name_;
    motor_state.can_id = can_id_;
    motor_state.motor_type = motor_type_;
    motor_state.motor_number = motor_id_;
    motor_state.velocity = real_current_;
    motor_state.current = given_current_;
    motor_state.temperature = temperature_;
    motor_state.total_angle = total_angle_;
}

} // namespace math_utilities