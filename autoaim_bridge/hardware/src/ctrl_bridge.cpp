#include "ctrl_bridge/ctrl_bridge.hpp"

// using namespace std::chrono;
// /**
//  * @brief 构造函数
//  * */
// CtrlBridge::CtrlBridge(const rclcpp::NodeOptions& options) : Node("ctrl_bridge", options)
// {
//   DelcareParams();  
//   //  init  data frame
//   receive_data_.SOF = 0XA5;
//   receive_data_.TOF = 0XA6;
//   send_data_.SOF = (0xA5);
//   send_data_.TOF = (0xA6);
  
//   serial_pub_ = this->create_publisher<rm_interfaces::msg::IMUReceive>("serial_data", rclcpp::SensorDataQoS());
  
//   // 创建一个订阅者订阅话题
//   cmd_vel_sub_ = this->create_subscription<rm_interfaces::msg::IMUSend>(cmd_vel_topic.c_str(), // 配置后面移植，先硬编代替
//                                                                       rclcpp::SystemDefaultsQoS(),
//                                                                       std::bind(&CtrlBridge::cmd_vel_callback,
//                                                                                 this,
//                                                                                 std::placeholders::_1));
//   static_pub_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
//   dynamic_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
//   try
//   {
//     RCLCPP_INFO(this->get_logger(), "serial_port:[%s]", serial_port_.c_str());
//     serial_.setPort(serial_port_);
//     serial_.setBaudrate(serial_baud_);
//     serial_.setFlowcontrol(serial::flowcontrol_none);
//     serial_.setParity(serial::parity_none); // default is parity_none
//     serial_.setStopbits(serial::stopbits_one);
//     serial_.setBytesize(serial::eightbits);
//     serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_);
//     serial_.setTimeout(time_out);
//     serial_.open();
//   }
//   catch (serial::IOException &e) // 抓取异常
//   {
//     RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
//     exit(0);
//   }
//   if (serial_.isOpen())
//   {
//     RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
//   } else {
//     RCLCPP_ERROR(this->get_logger(), "Unable to initial Serial port ");
//     exit(0);
//   }
//   std::thread([this]()
//               {
//           RCLCPP_INFO(this->get_logger(),"thread start");
//             this->process_receive(); })
//       .detach();
// }


// void CtrlBridge::process_receive() {           // 数据处理过程
//   RCLCPP_INFO(this->get_logger(), "receive start");
//   while (rclcpp::ok())
//   {
//     if (!serial_.isOpen())
//     {
//       RCLCPP_ERROR(this->get_logger(), "serial unopen");
//     }
//     // check head start   检查起始 数据帧头
//     uint8_t pdata[16] = {0x00};
//     if (serial_.read(pdata, 1) == 1 && pdata[0] == 0xa5)
//     {

//       if (serial_.read(pdata + 1, 15) != 15 || pdata[15] != 0xa6) {
//         continue;
//       }
//       // todo overflow
//       if (time_serial_cost_ / 10e6 < 0.5) {
//         auto stamp = this->now();
//         rm_interfaces::msg::IMUReceive data_buffer;
//         data_buffer.header.stamp = stamp;
//         data_buffer.header.frame_id = "middle_virtual_joint";
//         // std::memcpy(&data_buffer, pdata + 1, 14);
//         data_buffer.yaw_angle = *(float*)(pdata + 1);
//         data_buffer.pitch_angle = *(float*)(pdata + 5);
//         data_buffer.bullet_speed = *(float*)(pdata + 9);
//         data_buffer.target_mode = *(uint8_t*)(pdata + 13);
//         data_buffer.target_color = *(uint8_t*)(pdata + 14);

//         // RCLCPP_INFO(this->get_logger(), "yaw, pitch %f %f", data_buffer.yaw_angle, data_buffer.pitch_angle);

//         tf2::Quaternion q;
//         geometry_msgs::msg::TransformStamped ts_;
//         ts_.header.frame_id = "odom";
//         ts_.child_frame_id = "gyro";
//         ts_.transform.translation.z = 0.45;
//         ts_.transform.translation.x = 0;
//         ts_.transform.translation.y = 0;
//         q.setRPY(0, 0, 0);
//         ts_.transform.rotation.w = q.w();
//         ts_.transform.rotation.x = q.x();
//         ts_.transform.rotation.y = q.y();
//         ts_.transform.rotation.z = q.z();
//         ts_.header.stamp = stamp;
//         static_pub_->sendTransform(ts_);

//         q.setRPY(0, -data_buffer.pitch_angle * M_PI / 180.0, data_buffer.yaw_angle * M_PI / 180.0); 
//         ts_.child_frame_id = "middle_virtual_joint";
//         ts_.header.frame_id = "gyro";
//         ts_.transform.rotation.w = q.w();
//         ts_.transform.rotation.x = q.x();
//         ts_.transform.rotation.y = q.y();
//         ts_.transform.rotation.z = q.z();
//         ts_.transform.translation.x = 0;
//         ts_.transform.translation.y = 0;
//         ts_.transform.translation.z = 0;
//         ts_.header.stamp = stamp;
//         dynamic_pub_->sendTransform(ts_);
        
//         q.setRPY(-M_PI / 2, 0, -M_PI / 2);
//         ts_.child_frame_id = "camera";
//         ts_.header.frame_id = "middle_virtual_joint";
//         ts_.transform.rotation.w = q.w();
//         ts_.transform.rotation.x = q.x();
//         ts_.transform.rotation.y = q.y();
//         ts_.transform.rotation.z = q.z();
//         ts_.transform.translation.x = gim_xyz_coeff_[0] / 1000.0;
//         ts_.transform.translation.y = gim_xyz_coeff_[1] / 1000.0;
//         ts_.transform.translation.z = gim_xyz_coeff_[2] / 1000.0;
//         ts_.header.stamp = stamp;
//         static_pub_->sendTransform(ts_);

//         // q.setRPY(0, 0, 0);
//         // ts_.child_frame_id = "gun";
//         // ts_.header.frame_id = "camera";
//         // ts_.transform.translation.x = 0;
//         // ts_.transform.translation.y = -0.0428;
//         // ts_.transform.translation.z = 0;
//         // ts_.transform.rotation.w = q.w();
//         // ts_.transform.rotation.x = q.x();
//         // ts_.transform.rotation.y = q.y();
//         // ts_.transform.rotation.z = q.z();
//         // static_pub_->sendTransform(ts_);
        
//         serial_pub_->publish(data_buffer);
//       }
//     } else {
//       continue;
//     }
//   }
// }
// /**
//  * @brief 析构函数
//  * */
// CtrlBridge::~CtrlBridge()
// {
//   if (serial_.isOpen())
//     serial_.close();
// }

// /**
//  * @brief 收到话题数据的回调函数
//  * */
// void CtrlBridge::cmd_vel_callback(const rm_interfaces::msg::IMUSend cmd_vel)
// {
//   send_data_.cmd_ = cmd_vel.cmd;
//   send_data_.find_ = cmd_vel.find;
//   send_data_.id_ = cmd_vel.number;
//   send_data_.pitch_ = cmd_vel.pitch;
//   send_data_.yaw_ = cmd_vel.yaw;
//   // header
//   serial_.write((uint8_t *)(&send_data_.SOF), 1);
//   // cmd
//   uint8_t find = 1;
//   uint8_t temp = 0;
//   // RCLCPP_INFO(this->get_logger(), "CMd: %d",send_data_.cmd_);
//   serial_.write((uint8_t *)(&send_data_.yaw_), 4);
//   serial_.write((uint8_t *)(&send_data_.pitch_), 4);
//   serial_.write((uint8_t *)(&send_data_.find_), 1);
//   serial_.write((uint8_t *)(&send_data_.id_), 1);
//   serial_.write((uint8_t *)(&send_data_.cmd_), 1);
//   /// 注意此处有三个blank占位
//   serial_.write((uint8_t *)(&temp), 1);
//   serial_.write((uint8_t *)(&temp), 1);
//   serial_.write((uint8_t *)(&temp), 1);
//   // tail
//   serial_.write((uint8_t *)(&send_data_.TOF), 1);
// }


// void CtrlBridge::DelcareParams() {
//   this->declare_parameter<std::string>("serial_topic", "send_msg");
//   this->get_parameter("serial_topic", cmd_vel_topic);

//   this->declare_parameter<std::string>("serial_port_", "/dev/usb_serial");
//   this->get_parameter("serial_port_", serial_port_);

//   this->declare_parameter<std::int64_t>("serial_baud_", SERIAL_BAUD);
//   this->get_parameter("serial_baud_", serial_baud_);

//   // adjust to different car
//   // translation from camera to gimbal
//   gim_xyz_coeff_[0] = this->declare_parameter<double>("gim_xyz_coeff_x", 98.0);
//   gim_xyz_coeff_[1] = this->declare_parameter<double>("gim_xyz_coeff_y", 0);
//   gim_xyz_coeff_[2] = this->declare_parameter<double>("gim_xyz_coeff_z", 44.5);
// }

// rcl_interfaces::msg::SetParametersResult CtrlBridge::parametersCallBack(const std::vector<rclcpp::Parameter> & parameters) {
//     rcl_interfaces::msg::SetParametersResult result;
//     result.successful = true;
//     for (auto &param : parameters) {
//       if (param.get_name() == "serial_topic") {
//         cmd_vel_topic = param.as_string();
//       } else if (param.get_name() == "serial_port_") {
//         serial_port_ = param.as_string();
//       } else if (param.get_name() == "serial_baud_") {
//         serial_baud_ = param.as_int();
//       } 
//       else if (param.get_name() == "gim_xyz_coeff_x") {
//         gim_xyz_coeff_[0] = param.as_double();
//       } else if (param.get_name() == "gim_xyz_coeff_y") {
//         gim_xyz_coeff_[1] = param.as_double();
//       } else if (param.get_name() == "gim_xyz_coeff_z") {
//         gim_xyz_coeff_[2] = param.as_double();
//       }
//     }
//     return result;
// }
namespace helios_control {
    hardware_interface::CallbackReturn AutoAimBridge::on_init(const hardware_interface::HardwareInfo & info) {
        auto ret = Base::on_init(info);
        if (ret != hardware_interface::return_type::OK) {
            return ret;
        }
        serial_port_ = std::make_unique<serial::Serial>();
        serial_port_name_ = SERIAL_PORT_NAME;
        serial_port_baudrate_ = SERIAL_PORT_BAUDRATE;
        serial_port_timeout_ = SERIAL_PORT_TIMEOUT;
        if (serial_port_ == nullptr) {
            RCLCPP_ERROR(logger_, "Unable to allocate memory for serial port");
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn AutoAimBridge::on_configure(const rclcpp_lifecycle::State & previous_state) {

    }

    std::vector<hardware_interface::StateInterface> AutoAimBridge::export_state_interfaces() {

    }

    std::vector<hardware_interface::CommandInterface> AutoAimBridge::export_command_interfaces() {

    }

    hardware_interface::CallbackReturn AutoAimBridge::on_activate(const rclcpp_lifecycle::State & previous_state) {

    }

    hardware_interface::CallbackReturn AutoAimBridge::on_deactivate(const rclcpp_lifecycle::State & previous_state) {

    }

    hardware_interface::CallbackReturn AutoAimBridge::on_cleanup(const rclcpp_lifecycle::State & previous_state) {

    }

    hardware_interface::return_type AutoAimBridge::read(const rclcpp::Time & time, const rclcpp::Duration & period) {

    }

    hardware_interface::return_type AutoAimBridge::write(const rclcpp::Time & time, const rclcpp::Duration & period) {

    }

} // namespace helios_control