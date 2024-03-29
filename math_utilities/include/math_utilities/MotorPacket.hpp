//
// created by liuhan on 2023/9/25
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */
#pragma once

#include "controller_interface/controller_interface.hpp"
#include "helios_control_interfaces/msg/motor_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdint>
#include <stdint.h>
#include <string>
#include <sys/types.h>
#include <vector>

#define CAN_ID "can_id"
#define MOTOR_TYPE "motor_type"
#define MOTOR_ID "motor_id"
#define TOTAL_ANGLE "total_angle"
#define VELOCITY "velocity"
#define CURRENT "current"
#define TEMPERATURE "temperature"

namespace math_utilities {
class UniqueFlag {
public:
    uint8_t can_id;
    int16_t motor_type;
    uint8_t motor_id;

    bool operator<(const UniqueFlag& flag) const {
        if (can_id < flag.can_id) {
            return true;
        } else if (can_id == flag.can_id) {
            if (motor_type < flag.motor_type) {
                return true;
            } else if (motor_type == flag.motor_type) {
                if (motor_id < flag.motor_id) {
                    return true;
                }
            }
        }
        return false;
    }

    bool operator==(const UniqueFlag& flag) const {
        if (can_id == flag.can_id && motor_type == flag.motor_type && motor_id == flag.motor_id) {
            return true;
        }
        return false;
    }

    bool operator!=(const UniqueFlag& flag) const {
        if (can_id != flag.can_id || motor_type != flag.motor_type || motor_id != flag.motor_id) {
            return true;
        }
        return false;
    }

    bool operator>(const UniqueFlag& flag) const {
        if (can_id > flag.can_id) {
            return true;
        } else if (can_id == flag.can_id) {
            if (motor_type > flag.motor_type) {
                return true;
            } else if (motor_type == flag.motor_type) {
                if (motor_id > flag.motor_id) {
                    return true;
                }
            }
        }
        return false;
    }

    UniqueFlag& operator=(const UniqueFlag& flag) {
        can_id = flag.can_id;
        motor_type = flag.motor_type;
        motor_id = flag.motor_id;
        return *this;
    }

    UniqueFlag() {
        can_id = 0;
        motor_type = 0;
        motor_id = 0;
    }
};

class MotorPacket {
public:
    /**
    * @brief Construct a new Motor Packet object
    * 
    * @param pid_pos 
    * @param pid_vel 
    * @param pid_current 
    */
    MotorPacket(std::string motor_name);

    /**
     * @brief Get the moto measure object
     * 
     * @param state_interfaces 
     */
    static void get_moto_measure(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces, std::map<std::string, MotorPacket>& motor_map);
    /**
     * @brief Set the pid pos object
     * 
     * @param kp 
     * @param ki 
     * @param kd 
     * @param i_max 
     */
    void set_pid_pos(double kp, double ki, double kd, double i_max = 0);
    /**
     * @brief Set the pid vel object
     * 
     * @param kp 
     * @param ki 
     * @param kd 
     * @param i_max 
     */
    void set_pid_vel(double kp, double ki, double kd, double i_max = 0);
    /**
     * @brief Set the pid current object
     * 
     * @param kp 
     * @param ki 
     * @param kd 
     * @param i_max 
     */
    void set_pid_current(double kp, double ki, double kd, double i_max = 0);
    /**
     * @brief 
     * 
     * @param set_value 
     * @param real_value 
     * @return double 
     */
    double caculate_position_pid(double set_value, double real_value);
    /**
     * @brief 
     * 
     * @param set_value 
     * @param real_value 
     * @return double 
     */
    double caculate_speed_pid(double set_value, double real_value);
    /**
     * @brief 
     * 
     * @param set_value 
     * @param real_value 
     * @return double 
     */
    double caculate_current_pid(double set_value, double real_value);
    /**
     * @brief Set the state msg object
     * 
     * @param motor_state 
     */
    void set_state_msg(helios_control_interfaces::msg::MotorState& motor_state);
    /**
     * @brief Get the motor name object
     * 
     * @param motor_packet 
     */
    void calculate_motor_measure(MotorPacket motor_packet);

    std::string motor_name_;
    uint8_t can_id_;
    int motor_type_;
    uint8_t motor_id_;
    int16_t	mid_angle_;
    uint8_t motor_mode_;
    double value_;
    int32_t	round_cnt_;
    int64_t	total_angle_;
    uint8_t temperature_;
    int16_t real_current_;
    int16_t given_current_;
    int16_t	speed_rpm_;
};



} // namespace math_utilities