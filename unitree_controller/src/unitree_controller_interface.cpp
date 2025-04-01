#include "unitree_controller/unitree_controller_interface.hpp"

#include "controller_interface/helpers.hpp"


namespace unitree_controller
{

UnitreeControllerInterface::UnitreeControllerInterface()
: controller_interface::ControllerInterface(), 
  qJ_interface_(), 
  dqJ_interface_(), 
  tauJ_interface_(), 
  imu_orientation_interface_(), 
  imu_angular_velocity_interface_(), 
  imu_linear_acceleration_interface_(), 
  foot_force_sensor_interface_(),
  qJ_cmd_interface_(), 
  dqJ_cmd_interface_(), 
  tauJ_cmd_interface_(), 
  Kp_cmd_interface_(), 
  Kd_cmd_interface_(),
  states_(),
  commands_() {}

controller_interface::CallbackReturn UnitreeControllerInterface::on_init()
{
  RCLCPP_INFO(get_node()->get_logger(), "UnitreeControllerInterface::on_init() is called");
  try 
  {
    declare_parameters();
  }
  catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
UnitreeControllerInterface::command_interface_configuration() const
{
  RCLCPP_INFO(get_node()->get_logger(), "command_interface_configuration() is called");
  const std::vector<std::string> joint_names = this->get_joint_names();
  RCLCPP_INFO(get_node()->get_logger(), "this->get_joint_names() finished ");

  const std::vector<std::string> joint_command_interface_types = { hardware_interface::HW_IF_POSITION,
                                                                   hardware_interface::HW_IF_VELOCITY,
                                                                   hardware_interface::HW_IF_EFFORT,
                                                                   unitree_hardware::HW_IF_POSITION_GAIN,
                                                                   unitree_hardware::HW_IF_VELOCITY_GAIN};
  
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names.size() * joint_command_interface_types.size()); // Joint commands
  for (const auto & joint_name : joint_names)
  {
    for (const auto & interface_type : joint_command_interface_types)
    {
      // RCLCPP_INFO(get_node()->get_logger(), " binding joint_name: %s, interface_type: %s", joint_name.c_str(), interface_type.c_str());
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "finish command_interface_configuration(), now return ");

  return conf;
}

controller_interface::InterfaceConfiguration
UnitreeControllerInterface::state_interface_configuration() const
{
  RCLCPP_INFO(get_node()->get_logger(), "state_interface_configuration() called ");
  const std::vector<std::string> joint_names = this->get_joint_names();
  const std::vector<std::string> sensor_names = this->get_sensor_names();
  const std::vector<std::string> joint_state_interface_types = { hardware_interface::HW_IF_POSITION,
                                                                 hardware_interface::HW_IF_VELOCITY,
                                                                 hardware_interface::HW_IF_EFFORT};
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names.size() * joint_state_interface_types.size()  // Joint states
                      + 4 + 3 + 3  // Imu states (quat + gyro + acc)
                      + 4); // Foot force sensors 
  // Joint state
  for (const auto & joint_name : joint_names)
  {
    for (const auto & interface_type : joint_state_interface_types)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  // Imu state
  conf.names.push_back(sensor_names[0] + "/" + "orientation.x");
  conf.names.push_back(sensor_names[0] + "/" + "orientation.y");
  conf.names.push_back(sensor_names[0] + "/" + "orientation.z");
  conf.names.push_back(sensor_names[0] + "/" + "orientation.w");
  conf.names.push_back(sensor_names[0] + "/" + "angular_velocity.x");
  conf.names.push_back(sensor_names[0] + "/" + "angular_velocity.y");
  conf.names.push_back(sensor_names[0] + "/" + "angular_velocity.z");
  conf.names.push_back(sensor_names[0] + "/" + "linear_acceleration.x");
  conf.names.push_back(sensor_names[0] + "/" + "linear_acceleration.y");
  conf.names.push_back(sensor_names[0] + "/" + "linear_acceleration.z");
  // // Foot force sensor states
  // conf.names.push_back(sensor_names[1] + "/" + "force.z");
  // conf.names.push_back(sensor_names[2] + "/" + "force.z");
  // conf.names.push_back(sensor_names[3] + "/" + "force.z");
  // conf.names.push_back(sensor_names[4] + "/" + "force.z");
  return conf;
}

controller_interface::return_type UnitreeControllerInterface::update(
    const rclcpp::Time & time, const rclcpp::Duration & period) 
{ 

  // RCLCPP_INFO(get_node()->get_logger(), "UnitreeControllerInterface::update() called, take info from hardware interface -> send to unitree_controller");

  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return controller_interface::return_type::OK;
  }
  //  this is read() step
  // get joint state
  for (std::size_t i = 0 ; i < 12; ++i)
  {
    states_.qJ.coeffRef(i)   = qJ_interface_[i].get().get_value();
    states_.dqJ.coeffRef(i)  = dqJ_interface_[i].get().get_value();
    states_.tauJ.coeffRef(i) = tauJ_interface_[i].get().get_value();
  }
  // get Imu state
  states_.imu_orientation.x() = imu_orientation_interface_[0].get().get_value();
  states_.imu_orientation.y() = imu_orientation_interface_[1].get().get_value();
  states_.imu_orientation.z() = imu_orientation_interface_[2].get().get_value();
  states_.imu_orientation.w() = imu_orientation_interface_[3].get().get_value();
  for (std::size_t i = 0 ; i < 3; ++i) 
  {
    states_.imu_angular_velocity.coeffRef(i) = imu_angular_velocity_interface_[i].get().get_value();
  }
  for (std::size_t i = 0 ; i < 3; ++i) 
  {
    states_.imu_linear_acceleration.coeffRef(i) = imu_linear_acceleration_interface_[i].get().get_value();
  }
  // // get foor force sensor states 
  // for (std::size_t i = 0 ; i < 4; ++i) 
  // {
  //   states_.foot_force_sensor.coeffRef(i) = foot_force_sensor_interface_[i].get().get_value();
  // }

  // update controller, it will generate the control command
  // this update() was implemented inside unitree controller
  this->update(time, period, this->states_, this->commands_);


  // set joint commands, as a write() step
  for (std::size_t i = 0 ; i < 12; ++i)
  {
    qJ_cmd_interface_[i].get().set_value(commands_.qJ_cmd.coeff(i));
    dqJ_cmd_interface_[i].get().set_value(commands_.dqJ_cmd.coeff(i));
    tauJ_cmd_interface_[i].get().set_value(commands_.tauJ_cmd.coeff(i));
    Kp_cmd_interface_[i].get().set_value(commands_.Kp_cmd.coeff(i));
    Kd_cmd_interface_[i].get().set_value(commands_.Kd_cmd.coeff(i));
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
UnitreeControllerInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Failed to read parameters in UnitreeControllerInterface::on_configure");
    return ret;
  }
  RCLCPP_INFO(get_node()->get_logger(), "UnitreeControllerInterface::on_configure() completed successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
UnitreeControllerInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "UnitreeControllerInterface::on_activate() called");
  const std::vector<std::string> joint_names = this->get_joint_names();
  const std::vector<std::string> sensor_names = this->get_sensor_names();
  // Joint state interfaces
  qJ_interface_.clear();
  dqJ_interface_.clear();
  tauJ_interface_.clear();
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, joint_names, 
                                                    hardware_interface::HW_IF_POSITION, qJ_interface_)) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %d qJ_interface, got %ld.", 
      12, qJ_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, joint_names, 
                                                    hardware_interface::HW_IF_VELOCITY, dqJ_interface_)) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %d dqJ_interface, got %ld.", 
      12, dqJ_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, joint_names, 
                                                    hardware_interface::HW_IF_EFFORT, tauJ_interface_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %d tauJ_interface, got %ld.", 
      12, tauJ_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Imu state interfaces
  imu_orientation_interface_.clear();
  imu_angular_velocity_interface_.clear();
  imu_linear_acceleration_interface_.clear();
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names[0]}, 
                                               "orientation.x", imu_orientation_interface_);
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names[0]}, 
                                               "orientation.y", imu_orientation_interface_);
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names[0]}, 
                                               "orientation.z", imu_orientation_interface_);
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names[0]}, 
                                               "orientation.w", imu_orientation_interface_);
  if (imu_orientation_interface_.size() != 4) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %u imu_orientation_interface, got %lu.", 
      4, imu_orientation_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names[0]}, 
                                               "angular_velocity.x", imu_angular_velocity_interface_);
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names[0]}, 
                                               "angular_velocity.y", imu_angular_velocity_interface_);
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names[0]}, 
                                               "angular_velocity.z", imu_angular_velocity_interface_);
  if (imu_angular_velocity_interface_.size() != 3) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %u imu_angular_velocity_interface, got %lu.", 
      3, imu_orientation_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names[0]}, 
                                               "linear_acceleration.x", imu_linear_acceleration_interface_);
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names[0]}, 
                                               "linear_acceleration.y", imu_linear_acceleration_interface_);
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names[0]}, 
                                               "linear_acceleration.z", imu_linear_acceleration_interface_);
  if (imu_linear_acceleration_interface_.size() != 3) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %u imu_linear_acceleration_interface, got %lu.", 
      3, imu_linear_acceleration_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // // Foot force sensor interfaces
  // foot_force_sensor_interface_.clear();
  // for (std::size_t i = 0 ; i < 4; ++i) 
  // {
  //   controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names[i+1]}, 
  //                                                "force.z", foot_force_sensor_interface_);
  // }
  // if (foot_force_sensor_interface_.size() != 4) 
  // {
  //   RCLCPP_ERROR(
  //     get_node()->get_logger(), "Expected %u foot_force_interface, got %lu.", 
  //     4, foot_force_sensor_interface_.size());
  //   return controller_interface::CallbackReturn::ERROR;
  // }

  // Joint command interfaces
  qJ_cmd_interface_.clear();
  dqJ_cmd_interface_.clear();
  tauJ_cmd_interface_.clear();
  Kp_cmd_interface_.clear();
  Kd_cmd_interface_.clear();
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, joint_names, 
                                                    hardware_interface::HW_IF_POSITION, qJ_cmd_interface_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %u qJ_cmd_interface, got %lu.", 
      12, qJ_cmd_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, joint_names, 
                                                    hardware_interface::HW_IF_VELOCITY, dqJ_cmd_interface_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %u dqJ_cmd_interface, got %lu.", 
      12, dqJ_cmd_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, joint_names, 
                                                    hardware_interface::HW_IF_EFFORT, tauJ_cmd_interface_)) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %u tauJ_cmd_interface, got %lu.", 
      12, tauJ_cmd_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, joint_names, 
                                                    "Kp", Kp_cmd_interface_)) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %u Kp_cmd_interface, got %lu.", 
      12, Kp_cmd_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, joint_names, 
                                                    "Kd", Kd_cmd_interface_)) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %u Kp_cmd_interface, got %lu.", 
      12, Kd_cmd_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
UnitreeControllerInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (std::size_t i = 0; i < 12; ++i) 
  {
    qJ_cmd_interface_[i].get().set_value(UNITREE_LEGGED_SDK::PosStopF);
    dqJ_cmd_interface_[i].get().set_value(UNITREE_LEGGED_SDK::VelStopF);
    tauJ_cmd_interface_[i].get().set_value(0.0);
    Kp_cmd_interface_[i].get().set_value(0.0);
    Kd_cmd_interface_[i].get().set_value(0.0);
  }

  qJ_interface_.clear();
  dqJ_interface_.clear();
  tauJ_interface_.clear();
  imu_orientation_interface_.clear();
  imu_angular_velocity_interface_.clear();
  imu_linear_acceleration_interface_.clear();
  foot_force_sensor_interface_.clear();

  qJ_cmd_interface_.clear();
  dqJ_cmd_interface_.clear();
  tauJ_cmd_interface_.clear();
  Kp_cmd_interface_.clear();
  Kd_cmd_interface_.clear();

  release_interfaces();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
UnitreeControllerInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
UnitreeControllerInterface::on_error(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
UnitreeControllerInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace unitree_controller