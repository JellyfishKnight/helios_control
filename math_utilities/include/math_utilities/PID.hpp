// created by liuhan on 2023/9/21
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#pragma once

namespace math_utilities {

class PID {
public:
    PID();

    PID(double kp, double ki, double kd, double i_max);

    ~PID() = default;

    bool set_pid_params(double kp, double ki, double kd, double i_max);

    double pid_control(double set_value, double actual_value);

    double get_res_();
private:
    double kp_;
    double ki_;
    double kd_;
    double i_max_;

    double current_error_;
    double last_error_;
    double error_before_last_error_;

    double res_;
};


} // namespace math_utilities