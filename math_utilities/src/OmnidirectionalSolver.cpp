// created by liuhan on 2023/9/21
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#include "OmnidirectionalSolver.hpp"
#include <cmath>
#include <math.h>

namespace math_utilities {

void OmnidirectionalSolver::solve(geometry_msgs::msg::TwistStamped &twist_stamped) {
    // 底盘坐标系如下： 
    /*               battery
     *                  |+x  
     *              //  |  \\
     *            +y---------
     *              \\  |  //
     * 我们通过计算速度向量v(x, y)与单位向量(1, 0)的夹角来分解每个轮子上的速度
     */
    // linear velocity
    double v_lx = twist_stamped.twist.linear.x;
    double v_ly = twist_stamped.twist.linear.y;
    // angular velocity
    double v_az = twist_stamped.twist.angular.z;
    // get cos theta of velocity vector and (1, 0)
    double cos_theta = (v_lx  * 1 + v_ly * 0) / std::sqrt(v_lx * v_lx + v_ly * v_ly);
    // get angle
    double theta = std::acos(cos_theta);
    // because cos is even function, we need to specify the sign of theta
    if (v_ly < 0) {
        theta = -theta;
    }
    // get the velocity of each wheel
    front_left_v_ = v_lx * std::cos(theta + M_PI / 4) + v_ly * std::sin(theta + M_PI / 4) + v_az;
    front_right_v_ = v_lx * std::cos(theta - M_PI / 4) + v_ly * std::sin(theta - M_PI / 4) - v_az;
    back_left_v_ = v_lx * std::cos(theta - M_PI / 4) + v_ly * std::sin(theta - M_PI / 4) + v_az;
    back_right_v_ = v_lx * std::cos(theta + M_PI / 4) + v_ly * std::sin(theta + M_PI / 4) - v_az;
}

void OmnidirectionalSolver::get_target_values(
    double& front_left, double& front_right, double& back_left, double& back_right) {
    front_left = front_left_v_;
    front_right = front_right_v_;
    back_left = back_left_v_;
    back_right = back_right_v_;
}

} // namespace math_utilities