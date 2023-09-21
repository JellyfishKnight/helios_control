// created by liuhan on 2023/9/21
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#pragma once

#include "geometry_msgs/msg/twist_stamped.hpp"

namespace math_utilities {

class OmnidirectionalSolver {
public:
    OmnidirectionalSolver() = default;

    void solve(geometry_msgs::msg::TwistStamped &twist_stamped);

    void get_target_values(double& front_left, double& front_right, double& back_left, double& back_right);

    ~OmnidirectionalSolver() = default;
private:
    double front_left_v_;
    double front_right_v_;
    double back_left_v_;
    double back_right_v_;
};


} // namespace math_utilities