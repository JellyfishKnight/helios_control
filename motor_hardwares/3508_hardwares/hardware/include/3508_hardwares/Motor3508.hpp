#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"


namespace helios_control {
    
    class Motor3508 : public hardware_interface::ActuatorInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(helios_control::Motor3508);

        MOTOR3508_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

        MOTOR3508_PUBLIC
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        MOTOR3508_PUBLIC 
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        MOTOR3508_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        MOTOR3508_PUBLIC 
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

        MOTOR3508_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        MOTOR3508_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        MOTOR3508_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        MOTOR3508_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    private:
        std::vector<double> command_;
        std::vector<double> position_;

        
    };

}

