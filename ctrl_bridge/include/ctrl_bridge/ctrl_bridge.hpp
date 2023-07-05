#ifndef CTRL_BRIDGE_H
#define CTRL_BRIDGE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/static_transform_broadcaster.h>

#include <serial/serial.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "rm_interfaces/msg/receive_data.hpp"
#include "rm_interfaces/msg/send_data.hpp"

#include "chrono"
#include "thread"
#include "string"
#include "memory"

#define SERIAL_BAUD 115200

using namespace std;
using namespace chrono;
/*--------------------------------暂定协议-------------------------------------*/

/**---------------------------------------SEND DATA PROTOCOL--------------------------------------------**/
/**    ----------------------------------------------------------------------------------------------------
FIELD  |  A5  |  yaw  |  pitch  |  find  |   id   |   shoot  |   A6  |
       ----------------------------------------------------------------------------------------------------
BYTE   |   1  |   4   |    4    |    1   |    1   |    1     |   1  |
       ----------------------------------------------------------------------------------------------------
**/
/**---------------------------------------SEND DATA PROTOCOL--------------------------------------------**/


/**---------------------------------------RECEIVE DATA PROTOCOL-----------------------------------------------------------------------------**/
/**    -----------------------------------------------------------------------------------------------------------------------------------------
FIELD  |  head  |  yawAngle  |  pitchAngle  |  bullet  |  targetMode  |  targetColor |   A6  |
       ----------------------------------------------------------------------------------------------------
BYTE   |   1    |     4      |      4      |     4     |      1       |       1      |   1   |
------------------------------------------------------------------------------------------------------------------------------------------------
**/
/**---------------------------------------RECEIVE DATA PROTOCOL------------------------------------------------------------------------------**/
/**DESCRIPTION:
 * head: 0xA5
 * CmdID: Command ID
 * yawAngle: current yaw angle of pan-tilt
 * pitchAngle: current pitch angle of pan-tilt
 * yawSpeed: current spin speed of yaw direction
 * pitchSpeed: current pitch angle of pan-tilt
 * targetMode: the mode of vision task(AUTOSHOOT/ENERGY)
 * targetColor: blue or red enemy
 */

typedef enum {AUTOAIM, ENERGY, INIT} NodeType;

typedef struct __attribute__((__packed__)) {
    /*   head   */
    uint8_t SOF = 0xA5;
    /*   data   */
    float yaw_ = 0;
    float pitch_ = 0;
    int id_ = 0;
    uint8_t find_ = 0;
    uint8_t cmd_ = 0; // 1 shoot
    /*   tail   */
    uint8_t TOF = 0xA6;
} SendData;

typedef struct __attribute__((__packed__)) {    
    /*   head   */
    uint8_t SOF;
    /*   data   */
    float yaw_;             // 云台的yaw
    float pitch_;           // 云台的pitch
    float bullet_speed_;          // 子弹的射速
    uint8_t target_mode_;         // 发射模式(装甲板自瞄模式 或者 能量机关模式)
                                 // 0 是自瞄 1 是小能量机关 2 是大能量机关 但是目前1，2没有区别
    uint8_t target_color_;        // 目标颜色 0为红色，1为蓝色   
    /*   tail   */
    uint8_t TOF;
} ReceiveData;




class CtrlBridge : public rclcpp::Node {
private:
    /**
     * @brief 声明参数
    */
    void DelcareParams();
    /**
     * @brief 参数修改回调函数
    */
    rcl_interfaces::msg::SetParametersResult parametersCallBack(const std::vector<rclcpp::Parameter> & parameters);
    /**
     * @brief 接受器的回调函数
    */
    void cmd_vel_callback(const rm_interfaces::msg::SendData cmd_vel);
    /**
     * @brief 发送从串口读取的数据
    */
    void process_receive();

    //  serial
    rclcpp::Time time_last;
    serial::Serial serial_;
    std::string serial_port_;
    int serial_baud_;
    int serial_timeout_;
    //  topic name
    std::string cmd_vel_topic = "/cmd_vel";
    //  cmd_vel Subscription
    rclcpp::Subscription<rm_interfaces::msg::SendData>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<rm_interfaces::msg::ReceiveData>::SharedPtr serial_pub_;
    //  data_struct
    ReceiveData receive_data_;
    SendData send_data_;

    double gim_xyz_coeff_[3];

    geometry_msgs::msg::TransformStamped transform_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_pub_;

    double time_serial_cost_;
    double time_serial_start_;
    double last_time_serial_stamp_;
  public:
    CtrlBridge(const rclcpp::NodeOptions& options);

    ~CtrlBridge();
};

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(CtrlBridge);

#endif
