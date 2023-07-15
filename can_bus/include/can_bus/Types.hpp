#ifndef CAN_BUS_TYPES_HPP_
#define CAN_BUS_TYPES_HPP_

#include <string>
#include <math_utilities/LowPassFilter.hpp>
#include <math_utilities/IMUFilterBase.hpp>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>

namespace helios_control
{
    struct ActCoeff
    {
        double act2pos, act2vel, act2effort, pos2act, vel2act, effort2act, max_out, act2pos_offset, act2vel_offset,
            act2effort_offset, kp2act, kd2act;  // for MIT Cheetah motor
    };

    struct ActData
    {
        std::string name;
        std::string type;
        rclcpp::Time stamp;
        uint64_t seq;
        bool halted = false, need_calibration = false, calibrated = false, calibration_reading = false;
        uint16_t q_raw;
        int16_t qd_raw;
        uint8_t temp;
        int64_t q_circle;
        uint16_t q_last;
        double frequency;
        double pos, vel, effort;
        double cmd_pos, cmd_vel, cmd_effort, exe_effort;
        double offset;
        // For multiple cycle under absolute encoder (RoboMaster motor)
        math_utilities::LowPassFilter* lp_filter;
    };

    struct ImuData
    {
    rclcpp::Time time_stamp;
    std::string imu_name;
    double ori[4];
    double angular_vel[3], linear_acc[3];
    double angular_vel_offset[3];
    double ori_cov[9], angular_vel_cov[9], linear_acc_cov[9];
    double temperature, angular_vel_coeff, accel_coeff, temp_coeff, temp_offset;
    bool accel_updated, gyro_updated, camera_trigger;
    bool enabled_trigger;
    math_utilities::ImuFilterBase* imu_filter;
    };

    struct TofData
    {
    double strength;
    double distance;
    };

    struct CanDataPtr
    {
        std::unordered_map<std::string, ActCoeff>* type2act_coeffs_;
        std::unordered_map<int, ActData>* id2act_data_;
        std::unordered_map<int, ImuData>* id2imu_data_;
        std::unordered_map<int, TofData>* id2tof_data_;
    };
}  // namespace helios_control



#endif  // CAN_BUS_TYPES_HPP_