// created by liuhan on 2023/9/21
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink

#include "PID.hpp"

namespace math_utilities {

PID::PID() : 
    kp_(0.0), ki_(0.0), kd_(0.0), i_max_(0.0), 
    current_error_(0.0), last_error_(0.0), error_before_last_error_(0.0), res_(0.0) {}

PID::PID(double kp, double ki, double kd, double i_max) :
    kp_(kp), ki_(ki), kd_(kd), i_max_(i_max), 
    current_error_(0.0), last_error_(0.0), error_before_last_error_(0.0), res_(0.0) {}

bool PID::set_pid_params(double kp, double ki, double kd, double i_max) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    i_max_ = i_max;
    return true;
}

double PID::pid_control(double set_value, double actual_value) {
    current_error_ = set_value - actual_value;
    error_before_last_error_ += current_error_;

    if (error_before_last_error_ > i_max_)  error_before_last_error_ = i_max_;
    if (error_before_last_error_ < -i_max_) error_before_last_error_ = -i_max_;

    res_ = (kp_ * current_error_) +
        (ki_ * error_before_last_error_) +
        (kd_ * (current_error_ - last_error_));
    last_error_ = current_error_;
    return res_;
}

double PID::get_res_() {
    return res_;
}
} // namespace math_utilities