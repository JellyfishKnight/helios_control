#include "MotorPacket.hpp"
#include "PID.hpp"
#include <cstdint>
#include <utility>

namespace math_utilities {

MotorPacket::MotorPacket(std::string motor_name, int motor_mid_angle, 
                        PID& pid_pos, PID& pid_vel, PID& pid_current) :
    motor_name_(std::move(motor_name)), mid_angle_(motor_mid_angle), 
    pid_pos_(pid_pos), pid_vel_(pid_vel),
    pid_current_(pid_current) {}


void MotorPacket::get_moto_measure(std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
    // translate state_interfaces
    // we don't need temperature so we dropped it
    int temp_can_id, temp_motor_type, temp_motor_id;
    for (int i = 0; i < state_interfaces.size(); i++) {
        if (state_interfaces[i].get_prefix_name() == motor_name_) {
            // improve this: the joint name seems has marked each motor specificly
            // so we don't need their can_ids, motor_types and so on.
            if (state_interfaces[i].get_interface_name() == POSITION) {
                angle_ = static_cast<uint16_t>(state_interfaces[i].get_value());
            } else if (state_interfaces[i].get_interface_name() == VELOCITY) {
                real_current_ = static_cast<int16_t>(state_interfaces[i].get_value());
            } else if (state_interfaces[i].get_interface_name() == CURRENT) {
                given_current_ = static_cast<int16_t>(state_interfaces[i].get_value());
            } else if (state_interfaces[i].get_interface_name() == TEMPERATURE) {
                temperature_ = static_cast<uint8_t>(state_interfaces[i].get_value());
            }
        }
    }
    last_angle_ = angle_;
    last_total_angle_ = total_angle_;
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
    dif_angle_set_ = rpm;
    total_angle_set_ += rpm;
    pid_caculation_cnt_++;
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
    motor_state.position = angle_;
    motor_state.velocity = real_current_;
    motor_state.current = given_current_;
    motor_state.temperature = temperature_;
}

} // namespace math_utilities