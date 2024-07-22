#ifndef PCA9685_HARDWARE_INTERFACE__PCA9685_SYSTEM_HPP_
#define PCA9685_HARDWARE_INTERFACE__PCA9685_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "control_toolbox/pid.hpp"

#include "pca9685_hardware_interface/visibility_control.h"
#include <pca9685_hardware_interface/pca9685_comm.h>
#include <pca9685_hardware_interface/encoder_wj166.hpp>


namespace pca9685_hardware_interface
{

#define VELOCITY_PID_PARAMS_PREFIX "vel_"
#define POSITION_PID_PARAMS_PREFIX "pos_"

struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  explicit Joint(const std::string & name) 
  : joint_name(name)
  , reverse(false)
  {
    state = JointValue();
    command = JointValue();

    motor_id = 0;
    encoder_id = 0;
  }

  Joint() = default;

  std::string joint_name;
  JointValue state;
  JointValue command;

  uint8_t motor_id;
  uint8_t encoder_id;  

  bool reverse;

  control_toolbox::Pid vel_pid;
  
  std::function<void(double)> set_force;
};

class Pca9685SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Pca9685SystemHardware);

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  control_toolbox::Pid extractPID(std::string prefix, hardware_interface::ComponentInfo joint_info);
  bool extractReverse(hardware_interface::ComponentInfo joint_info);

private:
  std::map<std::string, Joint> hw_interfaces_;

  PiPCA9685::PCA9685 pca_; // 控制电机旋转

  encoder_wj166::Implementation encoder_wj166_; // 获取编码器值
};

}  // namespace pca9685_hardware_interface

#endif  // PCA9685_HARDWARE_INTERFACE__PCA9685_SYSTEM_HPP_
