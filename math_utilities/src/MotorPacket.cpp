#include "MotorPacket.hpp"
#include "PID.hpp"
#include <cstdint>
#include <utility>

namespace math_utilities {
MotorPacket::MotorPacket(std::string motor_name) : 
    motor_name_(std::move(motor_name)) {
    // init all member
    angle_ = 0;
    last_angle_ = 0;
    total_angle_ = 0;
    last_total_angle_ = 0;
    speed_rpm_ = 0;
    gimbal_angle_ = 0;
    dif_angle_ = 0;
    dif_angle_set_ = 0;
    total_angle_set_ = 0;
    buf_idx_ = 0;
    fited_angle_ = 0;
    msg_cnt_ = 0;
    can_send_ = 0;
    round_cnt_ = 0;
    temperature_ = 0;
    real_current_ = 0;
    given_current_ = 0;
    pid_caculation_cnt_ = 0;
}

MotorPacket::MotorPacket(std::string motor_name, int motor_mid_angle, 
                        PID& pid_pos, PID& pid_vel, PID& pid_current) :
    motor_name_(std::move(motor_name)), mid_angle_(motor_mid_angle), 
    pid_pos_(pid_pos), pid_vel_(pid_vel), 
    pid_current_(pid_current) {
    // init all member
    angle_ = 0;
    last_angle_ = 0;
    total_angle_ = 0;
    last_total_angle_ = 0;
    speed_rpm_ = 0;
    gimbal_angle_ = 0;
    dif_angle_ = 0;
    dif_angle_set_ = 0;
    total_angle_set_ = 0;
    buf_idx_ = 0;
    fited_angle_ = 0;
    msg_cnt_ = 0;
    can_send_ = 0;
    round_cnt_ = 0;
    temperature_ = 0;
    real_current_ = 0;
    given_current_ = 0;
    pid_caculation_cnt_ = 0;
}

void MotorPacket::calculate_motor_measure(MotorPacket motor_packet) {
    last_angle_ = angle_;
    last_total_angle_ = total_angle_;
    msg_cnt_++;
    angle_ = motor_packet.angle_;
    real_current_ = motor_packet.real_current_;
    given_current_ = motor_packet.given_current_;
    temperature_ = motor_packet.temperature_;
    speed_rpm_ = real_current_; //这里是因为两种电调对应位不一样的信息
    if (angle_ - last_angle_ > 4096)
        round_cnt_--;
    else if (angle_ - last_angle_ < -4096)
        round_cnt_++;
    total_angle_ = angle_ - mid_angle_ + round_cnt_ * 8192;

    //gimbal_angle代表云台相对初始位置的偏差角
    gimbal_angle_ = angle_ - mid_angle_;
    if (gimbal_angle_ > 8192 / 2) {
        gimbal_angle_ -= 8192;
    }
    else if (gimbal_angle_ < -8192 / 2) {
        gimbal_angle_ += 8192;
    }
    dif_angle_ = total_angle_ - last_total_angle_;
    // tesscan_++;
}

void MotorPacket::get_moto_measure(std::vector<hardware_interface::LoanedStateInterface>& state_interfaces, std::map<std::string, MotorPacket>& motor_map) {
    // translate state_interfaces
    std::map<std::string, MotorPacket> temp_map;
    for (auto& state : state_interfaces) {
        if (temp_map.find(state.get_prefix_name()) == temp_map.end()) {
            MotorPacket temp_packet(state.get_prefix_name());
            temp_map.emplace(std::pair<std::string, MotorPacket>(state.get_prefix_name(), temp_packet));
        }
        auto packet = temp_map.find(state.get_prefix_name());
        if (state.get_interface_name() == POSITION) {
            packet->second.angle_ = static_cast<uint16_t>(state.get_value());
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
            RCLCPP_FATAL(rclcpp::get_logger("MotorPacket"), "can_id: %d, motor_type: %d, motor_id: %d", temp_flag.can_id, temp_flag.motor_type, temp_flag.motor_id);
        } else {
            // RCLCPP_FATAL(rclcpp::get_logger("MotorPacket"), "can_id: %d, motor_type: %d, motor_id: %d", temp_flag.can_id, temp_flag.motor_type, temp_flag.motor_id);
            // RCLCPP_FATAL(rclcpp::get_logger("MotorPacket"), "can't find motor state: %s", motor_packet.first.c_str());
        }
    }
}

void MotorPacket::set_pid_pos(double kp, double ki, double kd, double i_max) {
    pid_pos_.set_pid_params(kp, ki, kd, i_max);
}

void MotorPacket::set_pid_vel(double kp, double ki, double kd, double i_max) {
    pid_vel_.set_pid_params(kp, ki, kd, i_max);
}

void MotorPacket::set_pid_current(double kp, double ki, double kd, double i_max) {
    pid_vel_.set_pid_params(kp, ki, kd, i_max);
}

double MotorPacket::set_motor_speed(int rpm) {
    if (rpm > 4000) {
        rpm = 4000;
    } else if (rpm < -4000) {
        rpm = -4000;
    }
    dif_angle_set_ = rpm;
    total_angle_set_ += rpm;
    pid_caculation_cnt_++;
    // RCLCPP_INFO(rclcpp::get_logger("resolver"), "total_angle_set: %d", total_angle_set_);
    // RCLCPP_INFO(rclcpp::get_logger("resolver"), "total_angle: %d", total_angle_);
    // // RCLCPP_INFO(rclcpp::get_logger("resolver"), "diff %d", total_angle_ - total_angle_set_);
    // // RCLCPP_INFO(rclcpp::get_logger("resolver"), "round_cnt: %d", round_cnt_);
    // RCLCPP_INFO(rclcpp::get_logger("resolver"), "rpm: %d", rpm);
    // speed circle
    pid_vel_.pid_control(pid_pos_.get_res_(), total_angle_ - last_total_angle_);
    // position circle
    if (pid_caculation_cnt_ >= 2) {
        pid_caculation_cnt_ = 0;
        pid_pos_.pid_control(total_angle_set_, total_angle_);
    }
    // limit pid_vel out in range from -16384 to 16384
    if (pid_vel_.get_res_() > 16384) {
        return 16384;
    } else if (pid_vel_.get_res_() < -16384) {
        return -16384;
    }
    return pid_vel_.get_res_();
}


double MotorPacket::set_motor_angle(int angle) {
    ///TODO: set motor angle
    double res;
    return res;
}

void MotorPacket::set_state_msg(helios_rs_interfaces::msg::MotorState& motor_state) {
    motor_state.full_name = motor_name_;
    motor_state.can_id = can_id_;
    motor_state.motor_type = motor_type_;
    motor_state.motor_number = motor_id_;
    motor_state.position = angle_;
    motor_state.velocity = real_current_;
    motor_state.current = given_current_;
    motor_state.temperature = temperature_;
}

} // namespace math_utilities