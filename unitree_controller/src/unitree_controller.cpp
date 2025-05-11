
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
  
  // Load trajectory from JSON
  if (!this->loadConfigFiles()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load trajectory files");
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

bool UnitreeController::loadConfigFiles() {
  bool success = true;
  for (size_t i = 0; i < this->config_files_.size(); ++i) {
    try {
      std::ifstream file(this->config_files_[i].path);
      if (!file.is_open()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to open the Json file %s", this->config_files_[i].path.c_str());
        success = false;
        continue;
      }

      nlohmann::json json_data;
      file >> json_data;
      file.close();
      for (const auto & row : json_data) {
        std::vector<double> joints;
        for (const auto & value : row) {
          joints.push_back(value.get<double>());
        }
        joint_trajectory[i].push_back(joints);
      }

      RCLCPP_INFO(get_node()->get_logger(), "Loaded trajectory '%s' with %zu points", 
                  this->config_files_[i].name.c_str(), joint_trajectory[i].size());

    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to load trajectory '%s': %s", 
                   this->config_files_[i].name.c_str(), e.what());
      success = false;
    }
  }
  return success;
}


controller_interface::return_type UnitreeController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period,
    const UnitreeStates & states, UnitreeCommands & commands) 
{
  RCLCPP_DEBUG(get_node()->get_logger(), "UnitreeController::update called, computed commands, ready to send");
  (void)period;
  (void)states;
  (void)time;
  // TODO: use the sitting on board position
  commands.qJ_cmd   = Eigen::Matrix<double, 12, 1>( this->joint_trajectory[4][0][0],
                                                    this->joint_trajectory[4][1][0],
                                                    this->joint_trajectory[4][2][0],
                                                    this->joint_trajectory[4][3][0],      
                                                    this->joint_trajectory[4][4][0],
                                                    this->joint_trajectory[4][5][0],
                                                    this->joint_trajectory[4][6][0],
                                                    this->joint_trajectory[4][7][0],
                                                    this->joint_trajectory[4][8][0],
                                                    this->joint_trajectory[4][9][0],
                                                    this->joint_trajectory[4][10][0],
                                                    this->joint_trajectory[4][11][0]);

  commands.dqJ_cmd  = sitting_down_controller_.dqJ_cmd();
  commands.tauJ_cmd = sitting_down_controller_.tauJ_cmd();
  commands.Kp_cmd   = sitting_down_controller_.Kp_cmd();
  commands.Kd_cmd   = sitting_down_controller_.Kd_cmd();


  this->control_mode_phuc_count += 1; 

  bool is_pushing = true;
  // use 5 iterations to track the trajectory
  int iteration_tracking = 5;

  // pusing mode
  if (is_pushing) {
    int trajectory_length = this->joint_trajectory[0].size();
    RCLCPP_INFO(get_node()->get_logger(), "Control mode phuc count: %d", this->control_mode_phuc_count);
    if(this->control_mode_phuc_count >= 2000 + trajectory_length * iteration_tracking){
      this->control_mode_phuc_count = 2000;
    }
  
    if(this->control_mode_phuc_count >= 2000 && this->control_mode_phuc_count < 2000 + trajectory_length * iteration_tracking) {
      // try to move the left front legs along the trajectory
      int current_index = static_cast<int>(floor((this->control_mode_phuc_count - 2000) / iteration_tracking));
      RCLCPP_INFO(get_node()->get_logger(), "current index: %d", current_index);

      commands.qJ_cmd.segment(3, 3) = Eigen::Vector3d(this->joint_trajectory[0][current_index][0],
                                                      this->joint_trajectory[0][current_index][1],
                                                      this->joint_trajectory[0][current_index][2]);
      commands.dqJ_cmd.segment(3, 3) = Eigen::Vector3d(this->joint_trajectory[1][current_index][0],
                                                      this->joint_trajectory[1][current_index][1],
                                                      this->joint_trajectory[1][current_index][2]);                                         
  
      RCLCPP_INFO(get_node()->get_logger(), "Moving right front leg to position: %f, %f, %f", 
                    commands.qJ_cmd[3], commands.qJ_cmd[4], commands.qJ_cmd[5]);
      return controller_interface::return_type::OK;
    }
  }
  // lifting mode
  else 
  {
    int trajectory_length = this->joint_trajectory[2].size();
    RCLCPP_INFO(get_node()->get_logger(), "Control mode phuc count: %d", this->control_mode_phuc_count);
    if(this->control_mode_phuc_count >= 2000 + trajectory_length * iteration_tracking){
      this->control_mode_phuc_count = 2000;
      this->is_lifting = !this->is_lifting;
    }
    if(this->control_mode_phuc_count > 2000 && this->control_mode_phuc_count < 2000 + trajectory_length * iteration_tracking) {
      int current_index = 0;
      if(!this->is_lifting) {
        current_index = static_cast<int>(floor((this->control_mode_phuc_count - 2000) / iteration_tracking));
        RCLCPP_INFO(get_node()->get_logger(), "current index not lifting: %d", current_index);
      }
      else {
        current_index = trajectory_length -1 - static_cast<int>(floor((this->control_mode_phuc_count - 2000) / iteration_tracking));
        RCLCPP_INFO(get_node()->get_logger(), "current index lifting: %d", current_index);
      }

      // try to move the left front legs along the trajectory
      commands.qJ_cmd.segment(3, 3) = Eigen::Vector3d(this->joint_trajectory[2][current_index][0],
                                                      this->joint_trajectory[2][current_index][1],
                                                      this->joint_trajectory[2][current_index][2]);
      commands.dqJ_cmd.segment(3, 3) = Eigen::Vector3d(this->joint_trajectory[3][current_index][0],
                                                      this->joint_trajectory[3][current_index][1],
                                                      this->joint_trajectory[3][current_index][2]);                                         
  
      RCLCPP_INFO(get_node()->get_logger(), "Moving right front leg to position: %f, %f, %f", 
                  commands.qJ_cmd[3], commands.qJ_cmd[4], commands.qJ_cmd[5]);
      return controller_interface::return_type::OK;
    }
  }

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