//
// Created by 17703 on 2022/3/17.
// edited by liuhan on 2023/9/10
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink

#ifndef KALMAN_RMKF_HPP
#define KALMAN_RMKF_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace math_utilities {

class EigenKalmanFilter{
public:
    EigenKalmanFilter(int state_param, int measure_param, int control_param = 0);
    void Init(int state_param, int measure_param, int control_param = 0);
    void Update(Eigen::VectorXf& measurement);
    Eigen::MatrixXf predict(const Eigen::MatrixXf& control = Eigen::MatrixXf());
    Eigen::MatrixXf correct(const Eigen::VectorXf& measurement);

    Eigen::VectorXf state_pre_;        // x_k_bar  k时刻的先验估计
    Eigen::VectorXf state_post_;       // x_k  k时刻的后验估计

    Eigen::MatrixXf trans_mat_;        // A 状态转移矩阵
    Eigen::MatrixXf control_mat_;      // Buk 控制矩阵
    Eigen::MatrixXf measure_mat_;      // H 测量转移矩阵
    Eigen::MatrixXf process_noise_;    // Q 过程激励噪声协方差矩阵
    Eigen::MatrixXf measure_noise_;    // R 测量噪声协方差矩阵
    Eigen::MatrixXf error_pre_;
    Eigen::MatrixXf gain_;             // K 滤波增益系数，卡尔曼系数
    Eigen::MatrixXf error_post_;

private:
    Eigen::MatrixXf temp1;
    Eigen::MatrixXf temp2;
    Eigen::MatrixXf temp3;
    Eigen::MatrixXf temp4;
};

} // namespace math_utilities

#endif