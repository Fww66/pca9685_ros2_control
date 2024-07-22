
#include "pca9685_hardware_interface/pca9685_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pca9685_hardware_interface
{
hardware_interface::CallbackReturn Pca9685SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  pca_.set_pwm_freq(50.0);

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
    hw_interfaces_[joint.name].reverse = extractReverse(joint);
    hw_interfaces_[joint.name].vel_pid = extractPID(VELOCITY_PID_PARAMS_PREFIX, joint);
    hw_interfaces_[joint.name].set_force = std::bind(&PiPCA9685::PCA9685::set_force, &pca_, hw_interfaces_[joint.name].motor_id, std::placeholders::_1);
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
  encoder_wj166_.update(); // 从编码器WJ166中读取所有位置速度

  for (auto & joint : hw_interfaces_)
  {
    double sign = joint.second.reverse ? -1.0 : 1.0;
    
    joint.second.state.position = sign * encoder_wj166_.get_position(joint.second.encoder_id);
    joint.second.state.velocity = sign * encoder_wj166_.get_velocity(joint.second.encoder_id);

    // RCLCPP_INFO(
    //   rclcpp::get_logger("Pca9685SystemHardware"),
    //   "state joint: %s, encoder_id: %d, position: %.3f, velocity: %.3f",
    //   joint.second.joint_name.c_str(),  joint.second.encoder_id, joint.second.state.position,
    //   joint.second.state.velocity/*,vel_measure,vel_filtered*/);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Pca9685SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    static std::map<int, control_toolbox::Pid::Gains> pid_gains = 
  {
    {0, { 0.005,    0.005,    0.001,   0.045,  -0.045, false}},
    {1, { 0.005,    0.005,    0.001,   0.05,  -0.05, false}},
    {2, { 0.006,    0.005,    0.00,   0.1,  -0.1, false}},
    {3, { 0.006,    0.005,    0.00,   0.1,  -0.1, false}},
    {4, { 0.006,    0.005,    0.00,   0.2,  -0.2, false}},
    {5, { 0.006,    0.005,    0.00,   0.2,  -0.2, false}},
    {6, { 0.007,    0.08,    0.00,   0.5,  -0.5, false}},
    {7, { 0.01,     0.05,    0.00,   0.99,  -0.99, false}},
    {8, { 0.02,     0.08,    0.00,   0.99,  -0.99, false}},
    {9, { 0.025,    0.08,    0.00,   0.99,  -0.99, false}}    
  };

  for (auto & joint : hw_interfaces_)
  {
    double sign = joint.second.reverse ? -1.0 : 1.0;

    int level = static_cast<int>(std::floor(std::fabs(joint.second.state.velocity)));    
    joint.second.vel_pid.setGains(pid_gains[level]);

    double goal_vel = joint.second.command.velocity;
    double cur_vel = joint.second.state.velocity;
    double error = goal_vel - cur_vel;
    uint64_t dt = period.nanoseconds();
    double cmd = joint.second.vel_pid.computeCommand(error, dt);
    if(cmd>-0.03 && cmd<0.03){cmd = 0;}
    else if(cmd>0){cmd = std::clamp(cmd, 0.03, 1.0);}
    else {cmd = std::clamp(cmd, -1.0, -0.03);}


    joint.second.set_force(sign*cmd);
    
    RCLCPP_INFO(
      rclcpp::get_logger("Pca9685SystemHardware"),
      "command joint: %s, motor_id: %d, dt: %.2f, goal_vel:%.2f, cmd:%.3f, cur_vel:%.3f,sign:%.3f", 
      joint.second.joint_name.c_str(), joint.second.motor_id, dt/1e9, joint.second.command.velocity, cmd, cur_vel,sign);
  }

  return hardware_interface::return_type::OK;
}

control_toolbox::Pid Pca9685SystemHardware::extractPID(std::string prefix, hardware_interface::ComponentInfo joint_info)
{
  double kp;
  double ki;
  double kd;
  double max_integral_error;
  double min_integral_error;
  bool antiwindup = false;

  if (joint_info.parameters.find(prefix + "kp") != joint_info.parameters.end()) {
    kp = std::stod(joint_info.parameters.at(prefix + "kp"));
  } else {
    kp = 0.0;
  }

  if (joint_info.parameters.find(prefix + "ki") != joint_info.parameters.end()) {
    ki = std::stod(joint_info.parameters.at(prefix + "ki"));
  } else {
    ki = 0.0;
  }

  if (joint_info.parameters.find(prefix + "kd") != joint_info.parameters.end()) {
    kd = std::stod(joint_info.parameters.at(prefix + "kd"));
  } else {
    kd = 0.0;
  }

  if (joint_info.parameters.find(prefix + "max_integral_error") != joint_info.parameters.end()) {
    max_integral_error = std::stod(joint_info.parameters.at(prefix + "max_integral_error"));
  } else {
    max_integral_error = std::numeric_limits<double>::max();
  }

  if (joint_info.parameters.find(prefix + "min_integral_error") != joint_info.parameters.end()) {
    min_integral_error = std::stod(joint_info.parameters.at(prefix + "min_integral_error"));
  } else {
    min_integral_error = std::numeric_limits<double>::min();
  }

  if (joint_info.parameters.find(prefix + "antiwindup") != joint_info.parameters.end()) {
    if (joint_info.parameters.at(prefix + "antiwindup") == "true" ||
      joint_info.parameters.at(prefix + "antiwindup") == "True")
    {
      antiwindup = true;
    }
  }

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("Pca9685SystemHardware"),
    "Setting kp = " << kp << "\t"
                    << " ki = " << ki << "\t"
                    << " kd = " << kd << "\t"
                    << " max_integral_error = " << max_integral_error);

  return control_toolbox::Pid(kp, ki, kd, max_integral_error, min_integral_error, antiwindup);
}

bool Pca9685SystemHardware::extractReverse(hardware_interface::ComponentInfo joint_info)
{
  bool reverse = false;

  if (joint_info.parameters.find("reverse") != joint_info.parameters.end()) 
  {
    if (joint_info.parameters.at("reverse") == "true" 
    || joint_info.parameters.at("reverse") == "True")
    {
      reverse = true;
    }
  }

  return reverse;
}

}  // namespace pca9685_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pca9685_hardware_interface::Pca9685SystemHardware, hardware_interface::SystemInterface)
