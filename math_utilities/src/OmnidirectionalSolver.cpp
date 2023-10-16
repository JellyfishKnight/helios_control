// created by liuhan on 2023/9/21
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#include "OmnidirectionalSolver.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <math.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace math_utilities {

void OmnidirectionalSolver::solve(geometry_msgs::msg::TwistStamped &twist_stamped, double yaw_diff) {
    // 底盘坐标系如下：                           虚拟底盘(和实际底盘相差yaw_diff角度):
    /*               battery                         ---+y    battery
     *                  |+x                           |  
     *              //  |  \\                  |______|______|+x
     *            +y---------                  |      |      |
     *              \\  |  //                         |
     *                                               ---
     * 我们先计算虚拟底盘上的运动，再将其转换到实际底盘上，转换仅仅是一个固定theta角的旋转
     * 假设轮子自身顺时针转动为-，逆时针转动为+
     */
    // linear velocity
    double v_lx = twist_stamped.twist.linear.x;
    double v_ly = twist_stamped.twist.linear.y;
    // angular velocity
    // z指向方向视角，顺时针为正，逆时针为负
    double v_z = twist_stamped.twist.angular.z;
    Eigen::Vector2d v_l(v_lx, v_ly);
    Eigen::Matrix2d R1;
    R1 << cos(M_PI_4), -sin(M_PI_4), sin(M_PI_4), cos(M_PI_4);
    // RCLCPP_INFO(rclcpp::get_logger("test"), "\n%f  %f\n%f  %f", R(0), R(1), R(2), R(3));
    // 旋转到虚拟底盘上
    Eigen::Vector2d v_r = R1 * v_l;
    Eigen::Matrix2d R2;
    R2 << cos(yaw_diff), -sin(yaw_diff), sin(yaw_diff), cos(yaw_diff);
    v_r = R2 * v_r; // 转换到实际底盘上
    front_left_v_ = v_r(0) + v_z;
    front_right_v_ = -v_r(1) + v_z;
    back_left_v_ = v_r(1) + v_z;
    back_right_v_ = -v_r(0) + v_z;
}

void OmnidirectionalSolver::get_target_values(
    double& front_left, double& front_right, double& back_left, double& back_right) {
    front_left = front_left_v_;
    front_right = front_right_v_;
    back_left = back_left_v_;
    back_right = back_right_v_;
}

} // namespace math_utilities