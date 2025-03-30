#include "unitree_controller/unitree_controller.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace unitree_controller
{

UnitreeController::UnitreeController()
: UnitreeControllerInterface(), 
  joint_names_({}), 
  sensor_names_({}),
  control_rate_(0), 
  control_period_(0),
  zero_torque_controller_(PDController::ZeroTorqueController()), 
  standing_up_controller_(PDController::StandingUpController()), 
  sitting_down_controller_(PDController::SittingDownController())
{
  // TODO: remove this for debugging 
  // set_contro_mode_srv_ = get_node()->create_service<unitree_msgs::srv::SetControlMode>(
  //     "set_control_mode", 
  //     std::bind(&UnitreeController::setControlModeCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void UnitreeController::declare_parameters() 
{
  // interfaces
  auto_declare<std::vector<std::string>>("joints", joint_names_);
  auto_declare<std::vector<std::string>>("sensors", sensor_names_);
  // node parameters
  auto_declare<int>("control_rate", 400);
}

controller_interface::CallbackReturn UnitreeController::read_parameters() 
{
  // Most probably will take information from ros2 parameters servers, and inherently come from .yaml files
  // interfaces
  RCLCPP_INFO(get_node()->get_logger(), "\n running read_parameters() \n");
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  sensor_names_ = get_node()->get_parameter("sensors").as_string_array();
  // node parameters
  control_rate_  = static_cast<double>(get_node()->get_parameter("control_rate").get_value<int>());

  if (joint_names_.size() != 12)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter has wrong size");
    return controller_interface::CallbackReturn::ERROR;
  }
  // if (sensor_names_.size() != 5)
  // {
  //   RCLCPP_ERROR(get_node()->get_logger(), "'sensors' parameter has wrong size");
  //   return controller_interface::CallbackReturn::ERROR;
  // }

  RCLCPP_INFO(get_node()->get_logger(), "Controller will be updated at %.2f Hz.", control_rate_);
  if (control_rate_ > 0.0)
  {
    control_period_ = 1.0 / control_rate_; // seconds
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'control_rate_' must be positive, got %lf.", control_rate_);
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<std::string> UnitreeController::get_joint_names() const {
  RCLCPP_INFO(get_node()->get_logger(), "get_joint_names() called ");
  return joint_names_;  
}

std::vector<std::string> UnitreeController::get_sensor_names() const {
  RCLCPP_INFO(get_node()->get_logger(), "get_sensor_names() called ");
  return sensor_names_;  
}


controller_interface::return_type UnitreeController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period,
    const UnitreeStates & states, UnitreeCommands & commands) 
{

  (void)period;
  (void)states;
  (void)time;
  // control_mode_ = *control_mode_rt_buffer_.readFromRT();
  this->control_mode_phuc_count += 1;
  if(this->control_mode_phuc_count >= 1000){
    this->control_mode_phuc_count = 0;
  }
  
  float alpha = 0.5 * (1.0 + std::sin(2.0 * M_PI * this->control_mode_phuc_count / 1000.0));  
  // interpolate for position between 2 modes
  commands.qJ_cmd   = (1.0 - alpha) * standing_up_controller_.qJ_cmd() + alpha * sitting_down_controller_.qJ_cmd();
  commands.dqJ_cmd  = standing_up_controller_.dqJ_cmd();
  commands.tauJ_cmd = standing_up_controller_.tauJ_cmd();
  commands.Kp_cmd   = standing_up_controller_.Kp_cmd();
  commands.Kd_cmd   = standing_up_controller_.Kd_cmd();
  return controller_interface::return_type::OK;

  // TODO: take mode from RT buffer
  // switch (control_mode_)
  // {
  //   case ControlMode::ZeroTorque: {
  //     commands.qJ_cmd   = zero_torque_controller_.qJ_cmd();
  //     commands.dqJ_cmd  = zero_torque_controller_.dqJ_cmd();
  //     commands.tauJ_cmd = zero_torque_controller_.tauJ_cmd();
  //     commands.Kp_cmd   = zero_torque_controller_.Kp_cmd();
  //     commands.Kd_cmd   = zero_torque_controller_.Kd_cmd();
  //     return controller_interface::return_type::OK;
  //     break;
  //   }
  //   case ControlMode::StandingUp: {
  //     commands.qJ_cmd   = standing_up_controller_.qJ_cmd();
  //     commands.dqJ_cmd  = standing_up_controller_.dqJ_cmd();
  //     commands.tauJ_cmd = standing_up_controller_.tauJ_cmd();
  //     commands.Kp_cmd   = standing_up_controller_.Kp_cmd();
  //     commands.Kd_cmd   = standing_up_controller_.Kd_cmd();
  //     return controller_interface::return_type::OK;
  //     break;
  //   }
  //   case ControlMode::SittingDown: {
  //     commands.qJ_cmd   = sitting_down_controller_.qJ_cmd();
  //     commands.dqJ_cmd  = sitting_down_controller_.dqJ_cmd();
  //     commands.tauJ_cmd = sitting_down_controller_.tauJ_cmd();
  //     commands.Kp_cmd   = sitting_down_controller_.Kp_cmd();
  //     commands.Kd_cmd   = sitting_down_controller_.Kd_cmd();
  //     return controller_interface::return_type::OK;
  //     break;
  //   }
  //   default: {
  //     RCLCPP_INFO(get_node()->get_logger(), "error in mode...############################");
  //     return controller_interface::return_type::ERROR;
  //     break;
  //   }
  // }
  
  return controller_interface::return_type::ERROR;
}

void UnitreeController::setControlModeCallback(const std::shared_ptr<unitree_msgs::srv::SetControlMode::Request> request,
                                               std::shared_ptr<unitree_msgs::srv::SetControlMode::Response> response) {
  (void) request;
  response->current_control_mode = FromControlModeToString(*control_mode_rt_buffer_.readFromNonRT());
  // control_mode_rt_buffer_.writeFromNonRT(FromStringToControlMode(request->control_mode));
  control_mode_rt_buffer_.writeFromNonRT(ControlMode::ZeroTorque);
  response->accept = true;
}

} // namespace unitree_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  unitree_controller::UnitreeController, 
  controller_interface::ControllerInterface)