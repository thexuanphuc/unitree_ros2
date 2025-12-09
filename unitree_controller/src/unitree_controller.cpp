#include "unitree_controller/unitree_controller.hpp"

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
  sitting_down_controller_(PDController::SittingDownController()),
  mpc_start_push_(MpcStartPush::Zero()), // Initialize MpcStartPush
  mpc_turning_(MpcStartTurning::Zero()) // Initialize MpcStartTurning
{
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
  RCLCPP_INFO(get_node()->get_logger(), "\n running read_parameters() \n");
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  sensor_names_ = get_node()->get_parameter("sensors").as_string_array();
  control_rate_ = static_cast<double>(get_node()->get_parameter("control_rate").get_value<int>());

  if (joint_names_.size() != 12)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter has wrong size");
    return controller_interface::CallbackReturn::ERROR;
  }

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
  if(this->is_turning_){
    if(!mpc_turning_.load_config()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to load trajectory files for turning");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  else{
    // Load trajectory using MpcStartPush
    if(!mpc_start_push_.load_config()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load trajectory files for pushing");
    return controller_interface::CallbackReturn::ERROR;
  }

  }
  RCLCPP_INFO(get_node()->get_logger(), "Loaded trajectory files successfully");
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
  if(this->is_turning_){
    // Compute desired trajectory using MpcStartTurning and unpack tuple
    auto [qJ_cmd, dqJ_cmd, tauJ_cmd, Kp_cmd, Kd_cmd, mode] = mpc_turning_.compute_desired_trajectory();
    // RCLCPP_INFO(get_node()->get_logger(), "UnitreeController::update called, the mode is %d, the qj is %f", mode, qJ_cmd);
    commands.qJ_cmd = qJ_cmd;
    commands.dqJ_cmd = dqJ_cmd;
    commands.tauJ_cmd = tauJ_cmd;
    commands.Kp_cmd = Kp_cmd;
    commands.Kd_cmd = Kd_cmd;
  }
  else{
    // Compute desired trajectory using MpcStartPush and unpack tuple
    auto [qJ_cmd, dqJ_cmd, tauJ_cmd, Kp_cmd, Kd_cmd, mode] = mpc_start_push_.compute_desired_trajectory();
    // RCLCPP_INFO(get_node()->get_logger(), "UnitreeController::update called, the mode is %d, the qj is %f", mode, qJ_cmd);
    commands.qJ_cmd = qJ_cmd;
    commands.dqJ_cmd = dqJ_cmd;
    commands.tauJ_cmd = tauJ_cmd;
    commands.Kp_cmd = Kp_cmd;
    commands.Kd_cmd = Kd_cmd;
  }

  return controller_interface::return_type::OK;
}

void UnitreeController::setControlModeCallback(
    const std::shared_ptr<unitree_msgs::srv::SetControlMode::Request> request,
    std::shared_ptr<unitree_msgs::srv::SetControlMode::Response> response) 
{
  (void) request;
  response->current_control_mode = FromControlModeToString(*control_mode_rt_buffer_.readFromNonRT());
  control_mode_rt_buffer_.writeFromNonRT(ControlMode::ZeroTorque);
  response->accept = true;
}

} // namespace unitree_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  unitree_controller::UnitreeController, 
  controller_interface::ControllerInterface)