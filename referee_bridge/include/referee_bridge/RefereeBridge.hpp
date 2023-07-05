#ifndef REFEREEE_BRIDGE
#define REFEREEE_BRIDGE

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <Referee.h>
#include <CRC.h>

#include <rm_interfaces/msg/game_robot_hp.hpp>
#include <rm_interfaces/msg/power_heat_data.hpp>
#include <rm_interfaces/msg/shoot_data.hpp>

typedef struct __packed
{
    uint8_t sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
    uint16_t cmd_id;
    uint8_t data[256]{};
    bool CheckHeaderCRC8()
    {
      return Verify_CRC8_Check_Sum((uint8_t *)this, GetHeaderLength());
    }
    bool CheckPackCRC16()
    {
      return Verify_CRC16_Check_Sum((uint8_t *)this, GetLength());
    }
    uint16_t CheckTail()
    {
      uint16_t tail = *(uint16_t *)((uint8_t *)this + GetLength() - 2);
      return tail;
    }
    void AddCRC()
    {
      Append_CRC8_Check_Sum((uint8_t *)this, GetHeaderLength());
      Append_CRC16_Check_Sum((uint8_t *)this, GetLength());
    }
    size_t GetHeaderLength() { return 5; }
    size_t GetLength() { return 7 + data_length + 2; }

    
} FrameBuffer;


class RefereeBridge : public rclcpp::Node {
private:
    serial::Serial serial_port_;
    std::string serial_port_name_;
    int serial_port_baudrate_;
    int serial_port_timeout_;

    FrameBuffer header_receive_buffer_;
    // Publishers
    rclcpp::Publisher<rm_interfaces::msg::GameRobotHP>::SharedPtr game_robot_hp_pub_;
    rclcpp::Publisher<rm_interfaces::msg::PowerHeatData>::SharedPtr power_heat_data_pub_;
    rclcpp::Publisher<rm_interfaces::msg::ShootData>::SharedPtr shoot_data_pub_;

    int SOF_ = 0xA5;
    int TOF_ = 0xA6;

    void InitSerial();

    void DelcareParams();  

    void ProcessFuntion();

    void RegisterTopics();
public:
    RefereeBridge(const rclcpp::NodeOptions& options);
    ~RefereeBridge() = default;
};


#endif