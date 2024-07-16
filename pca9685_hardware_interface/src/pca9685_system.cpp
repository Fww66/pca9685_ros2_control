
#include "pca9685_hardware_interface/pca9685_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pca9685_hardware_interface
{
hardware_interface::CallbackReturn Pca9685SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  encoder_wj166_ = std::make_shared<encoder_wj166::Implementation>();

  // pca.set_pwm_freq(50.0);

  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Pca9685System has one command interface on each output
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    hw_interfaces_[joint.name] = Joint(joint.name);
    hw_interfaces_[joint.name].motor_id = std::stoi(joint.parameters.at("motor_id"));
    hw_interfaces_[joint.name].encoder_id = std::stoi(joint.parameters.at("encoder_id"));
    hw_interfaces_[joint.name].filter.configure(0.5);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> Pca9685SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.second.joint_name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.second.joint_name, hardware_interface::HW_IF_VELOCITY, &joint.second.state.velocity));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Pca9685SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (auto & joint : hw_interfaces_)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.second.joint_name, hardware_interface::HW_IF_VELOCITY, &joint.second.command.velocity));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & joint : hw_interfaces_)
  {
    joint.second.state.position = 0.0;
    joint.second.state.velocity = 0.0;

    joint.second.command.velocity = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Pca9685SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto & joint : hw_interfaces_)
  {
    joint.second.state.position = encoder_wj166_->get_position(joint.second.encoder_id);
    // joint.second.state.velocity = encoder_wj166_->get_velocity(joint.second.encoder_id);
    joint.second.filter.update(encoder_wj166_->get_velocity(joint.second.encoder_id), joint.second.state.velocity);

    RCLCPP_INFO(
      rclcpp::get_logger("Pca9685SystemHardware"),
      "state joint: %s, encoder_id: %d, position: %.3f, velocity: %.3f", joint.second.joint_name.c_str(),  joint.second.encoder_id, joint.second.state.position, joint.second.state.velocity);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Pca9685SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto & joint : hw_interfaces_)
  {
    // pca.set_force(joint.second.motor_id, joint.second.command.velocity);

    RCLCPP_INFO(
      rclcpp::get_logger("Pca9685SystemHardware"),
      "command joint: %s, motor_id: %d, velocity: %.3f", joint.second.joint_name.c_str(),  joint.second.motor_id, joint.second.command.velocity);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace pca9685_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pca9685_hardware_interface::Pca9685SystemHardware, hardware_interface::SystemInterface)
