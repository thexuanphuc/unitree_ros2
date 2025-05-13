
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
  this->all_trajectory_length = 0;

  // load position trajectory
  for (size_t i = 0; i < this->config_pos_files_.size(); ++i) {
    try {
      std::ifstream file(this->config_pos_files_[i].path);
      if (!file.is_open()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to open the Json file %s", this->config_pos_files_[i].path.c_str());
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
                  this->config_pos_files_[i].name.c_str(), joint_trajectory[i].size());

    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to load trajectory '%s': %s", 
                   this->config_pos_files_[i].name.c_str(), e.what());
      success = false;
    }
  }

  // load velocity trajectory
  for (size_t i = 0; i < this->config_vel_files_.size(); ++i) {
    try {
      std::ifstream file(this->config_vel_files_[i].path);
      if (!file.is_open()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to open the Json file %s", this->config_vel_files_[i].path.c_str());
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
        joint_velocity_trajectory[i].push_back(joints);
      }

      RCLCPP_INFO(get_node()->get_logger(), "Loaded trajectory '%s' with %zu points", 
                  this->config_vel_files_[i].name.c_str(), joint_velocity_trajectory[i].size());

    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to load trajectory '%s': %s", 
                   this->config_vel_files_[i].name.c_str(), e.what());
      success = false;
    }
  }

  this->count_accummulation.push_back(1000 * 1); // 1 because we have only 1 state on the trajectory
  this->count_accummulation.push_back(this->count_accummulation.back() + this->joint_trajectory[1].size());
  this->count_accummulation.push_back(this->count_accummulation.back() + this->joint_trajectory[2].size());
  this->count_accummulation.push_back(this->count_accummulation.back() + this->joint_trajectory[3].size());
  this->count_accummulation.push_back(this->count_accummulation.back() + this->joint_trajectory[4].size());
  this->count_accummulation.push_back(this->count_accummulation.back() + this->push_times * this->joint_trajectory[5].size());
  this->count_accummulation.push_back(this->count_accummulation.back() + this->joint_trajectory[6].size()); 
  this->count_accummulation.push_back(this->count_accummulation.back() + 3000);  // 3000 / 400 = 7.5 seconds wait for 7.5 seconds
  this->max_count = this->count_accummulation.back() * this->iteration_tracking;
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

  // order of legs: [q_FL, q_FR, q_RL, q_RR]
  // for now we just play with kinematics trajectory
  commands.tauJ_cmd = sitting_down_controller_.tauJ_cmd();
  commands.Kp_cmd = sitting_down_controller_.Kp_cmd();
  commands.Kd_cmd = sitting_down_controller_.Kd_cmd();
  this->control_mode_phuc_count += 1;

  if (this->control_mode_phuc_count >= this->max_count) {
    // Reset the action
    RCLCPP_INFO(get_node()->get_logger(), "############################3 to the end, reset to mode 3, the count should be %d", this->count_accummulation[3]);
    this->control_mode_phuc_count = this->count_accummulation[3] * this->iteration_tracking;
  }
  this->cur_index = static_cast<int>(floor((this->control_mode_phuc_count ) / this->iteration_tracking));

  size_t mode = 0;
  while (mode < this->count_accummulation.size() && 
          this->cur_index >= this->count_accummulation[mode]) {
    ++mode;
    RCLCPP_DEBUG(get_node()->get_logger(), "the mode was changed %ld", mode);
  }

  // moving mode 
  // 0 is just normal sitting on board
  // 1 is move the body to the right (move 4 legs)           
  // 2 is move the left front leg into center
  // 3 is move the body back to the left
  // 4 is move the right leg to the floor
  // 5 is push the front right leg on the floor N times
  // 6 is move the front right leg back to the skateboard
  // 7 just wait for some seconds before the next pushing

  RCLCPP_INFO(get_node()->get_logger(), "the mode is %ld", mode);

  switch (mode) {
    case 0: // Mode 0: Normal sitting on board
    {
      commands.qJ_cmd = Eigen::Matrix<double, 12, 1>(
        this->joint_trajectory[mode][0][0], this->joint_trajectory[mode][1][0],
        this->joint_trajectory[mode][2][0], this->joint_trajectory[mode][3][0],
        this->joint_trajectory[mode][4][0], this->joint_trajectory[mode][5][0],
        this->joint_trajectory[mode][6][0], this->joint_trajectory[mode][7][0],
        this->joint_trajectory[mode][8][0], this->joint_trajectory[mode][9][0],
        this->joint_trajectory[mode][10][0], this->joint_trajectory[mode][11][0]
      );
      commands.dqJ_cmd = Vector12d::Zero();
      RCLCPP_INFO(get_node()->get_logger(), "normal sitting on board");
      return controller_interface::return_type::OK;
      break;
    }

    case 1: // Mode 1: Move body to the right
    {
      int case_index = this->cur_index - this->count_accummulation[mode-1];
      // RCLCPP_INFO(get_node()->get_logger(), "out side index %d", this->cur_index);
      // RCLCPP_INFO(get_node()->get_logger(), "this->count_accummulation[mode] %d", this->count_accummulation[mode]);
      // RCLCPP_INFO(get_node()->get_logger(), "this->count_accummulation[mode-1] %d", this->count_accummulation[mode-1]);

      // RCLCPP_INFO(get_node()->get_logger(), "the case index is %d", case_index);

      commands.qJ_cmd = Eigen::Matrix<double, 12, 1>(
        this->joint_trajectory[mode][case_index][0], this->joint_trajectory[mode][case_index][1],
        this->joint_trajectory[mode][case_index][2], this->joint_trajectory[mode][case_index][3],
        this->joint_trajectory[mode][case_index][4], this->joint_trajectory[mode][case_index][5],
        this->joint_trajectory[mode][case_index][6], this->joint_trajectory[mode][case_index][7],
        this->joint_trajectory[mode][case_index][8], this->joint_trajectory[mode][case_index][9],
        this->joint_trajectory[mode][case_index][10], this->joint_trajectory[mode][case_index][11]
      );
      commands.dqJ_cmd = Vector12d::Zero();
      RCLCPP_DEBUG(get_node()->get_logger(), "Moving body to the right");
      return controller_interface::return_type::OK;
      break;
    }

    case 2: // Mode 2: Move the left leg to center
    {
      int case_index = this->cur_index - this->count_accummulation[mode-1];
      RCLCPP_INFO(get_node()->get_logger(), "the case index is %d", case_index);
      // first keep others joint position unchanges
      commands.qJ_cmd.segment(3, 9) = Eigen::Matrix<double, 9, 1>(
        this->joint_trajectory[mode-1].back()[3],
        this->joint_trajectory[mode-1].back()[4], this->joint_trajectory[mode-1].back()[5],
        this->joint_trajectory[mode-1].back()[6], this->joint_trajectory[mode-1].back()[7],
        this->joint_trajectory[mode-1].back()[8], this->joint_trajectory[mode-1].back()[9],
        this->joint_trajectory[mode-1].back()[10], this->joint_trajectory[mode-1].back()[11]
      );
      commands.dqJ_cmd = Vector12d::Zero();
      // now apply new command
      commands.qJ_cmd.segment(0, 3) = Eigen::Vector3d(  this->joint_trajectory[mode][case_index][0],
                                                        this->joint_trajectory[mode][case_index][1],
                                                        this->joint_trajectory[mode][case_index][2]);
      commands.dqJ_cmd.segment(0, 3) = Eigen::Vector3d( this->joint_velocity_trajectory[0][case_index][0],
                                                        this->joint_velocity_trajectory[0][case_index][1],
                                                        this->joint_velocity_trajectory[0][case_index][2]);                                         

      RCLCPP_DEBUG(get_node()->get_logger(), "Moving body to the left");
      return controller_interface::return_type::OK;
      break;
    }

    case 3: //move the body back to the left
    {
      int case_index = this->cur_index - this->count_accummulation[mode-1];
      RCLCPP_INFO(get_node()->get_logger(), "the case index is %d", case_index);

      commands.qJ_cmd = Eigen::Matrix<double, 12, 1>(
        this->joint_trajectory[mode][case_index][0], this->joint_trajectory[mode][case_index][1],
        this->joint_trajectory[mode][case_index][2], this->joint_trajectory[mode][case_index][3],
        this->joint_trajectory[mode][case_index][4], this->joint_trajectory[mode][case_index][5],
        this->joint_trajectory[mode][case_index][6], this->joint_trajectory[mode][case_index][7],
        this->joint_trajectory[mode][case_index][8], this->joint_trajectory[mode][case_index][9],
        this->joint_trajectory[mode][case_index][10], this->joint_trajectory[mode][case_index][11]
      );
      commands.dqJ_cmd = Vector12d::Zero();
      RCLCPP_DEBUG(get_node()->get_logger(), "Moving body to the left: %f, %f, %f", 
                                              commands.qJ_cmd[0], commands.qJ_cmd[1], commands.qJ_cmd[2]);
      return controller_interface::return_type::OK;
      break;
    }

    case 4: //move the right leg to the floor
    {
      int case_index = this->cur_index - this->count_accummulation[mode-1];
      // RCLCPP_INFO(get_node()->get_logger(), "the case index is %d", case_index);
      // RCLCPP_INFO(get_node()->get_logger(), "out side index %d", this->cur_index);
      // RCLCPP_INFO(get_node()->get_logger(), "this->count_accummulation[mode] %d", this->count_accummulation[mode]);
      // RCLCPP_INFO(get_node()->get_logger(), "this->joint_trajectory[mode] %d", this->joint_trajectory[mode].size());
      // first keep others joint position unchanges
      commands.qJ_cmd = Eigen::Matrix<double, 12, 1>(
        this->joint_trajectory[mode-1].back()[0], this->joint_trajectory[mode-1].back()[1],
        this->joint_trajectory[mode-1].back()[2], this->joint_trajectory[mode-1].back()[3],
        this->joint_trajectory[mode-1].back()[4], this->joint_trajectory[mode-1].back()[5],
        this->joint_trajectory[mode-1].back()[6], this->joint_trajectory[mode-1].back()[7],
        this->joint_trajectory[mode-1].back()[8], this->joint_trajectory[mode-1].back()[9],
        this->joint_trajectory[mode-1].back()[10], this->joint_trajectory[mode-1].back()[11]
      );
      commands.dqJ_cmd = Vector12d::Zero();
      // now apply new command
      commands.qJ_cmd.segment(3, 3) = Eigen::Vector3d(  this->joint_trajectory[mode][case_index][0],
                                                        this->joint_trajectory[mode][case_index][1],
                                                        this->joint_trajectory[mode][case_index][2]);
      commands.dqJ_cmd.segment(3, 3) = Eigen::Vector3d( this->joint_velocity_trajectory[1][case_index][0],
                                                        this->joint_velocity_trajectory[1][case_index][1],
                                                        this->joint_velocity_trajectory[1][case_index][2]);                                         

      RCLCPP_DEBUG(get_node()->get_logger(), "Moving right front leg to position: %f, %f, %f", 
                                              commands.qJ_cmd[3], commands.qJ_cmd[4], commands.qJ_cmd[5]);
      return controller_interface::return_type::OK;
      break;
    }

    case 5:  // Push the front right leg on the floor N times, it start to fuck up the front left
    {
      int case_index = (this->cur_index - this->count_accummulation[mode - 1]) % this->joint_trajectory[mode].size();
      RCLCPP_INFO(get_node()->get_logger(), "the case index is %d", case_index);

      // first keep others joint position unchanges
      commands.qJ_cmd = Eigen::Matrix<double, 12, 1>(
        this->joint_trajectory[mode-2].back()[0], this->joint_trajectory[mode-2].back()[1],
        this->joint_trajectory[mode-2].back()[2], this->joint_trajectory[mode-2].back()[3],
        this->joint_trajectory[mode-2].back()[4], this->joint_trajectory[mode-2].back()[5],
        this->joint_trajectory[mode-2].back()[6], this->joint_trajectory[mode-2].back()[7],
        this->joint_trajectory[mode-2].back()[8], this->joint_trajectory[mode-2].back()[9],
        this->joint_trajectory[mode-2].back()[10], this->joint_trajectory[mode-2].back()[11]
      );
      commands.dqJ_cmd = Vector12d::Zero();
      // now apply new command
      commands.qJ_cmd.segment(3, 3) = Eigen::Vector3d(  this->joint_trajectory[mode][case_index][0],
                                                        this->joint_trajectory[mode][case_index][1],
                                                        this->joint_trajectory[mode][case_index][2]);
      commands.dqJ_cmd.segment(3, 3) = Eigen::Vector3d( this->joint_velocity_trajectory[2][case_index][0],
                                                        this->joint_velocity_trajectory[2][case_index][1],
                                                        this->joint_velocity_trajectory[2][case_index][2]);                                         

      RCLCPP_DEBUG(get_node()->get_logger(), "Moving right front leg to position: %f, %f, %f", 
                                              commands.qJ_cmd[3], commands.qJ_cmd[4], commands.qJ_cmd[5]);
      return controller_interface::return_type::OK;
      break;
    }

    case 6: // take the front right leg back to the skateboard
    {
      int case_index = this->cur_index - this->count_accummulation[mode - 1];
      RCLCPP_INFO(get_node()->get_logger(), "the case index is %d", case_index);
      // first keep others joint position unchanges
      commands.qJ_cmd = Eigen::Matrix<double, 12, 1>(
        this->joint_trajectory[mode-3].back()[0], this->joint_trajectory[mode-3].back()[1],
        this->joint_trajectory[mode-3].back()[2], this->joint_trajectory[mode-3].back()[3],
        this->joint_trajectory[mode-3].back()[4], this->joint_trajectory[mode-3].back()[5],
        this->joint_trajectory[mode-3].back()[6], this->joint_trajectory[mode-3].back()[7],
        this->joint_trajectory[mode-3].back()[8], this->joint_trajectory[mode-3].back()[9],
        this->joint_trajectory[mode-3].back()[10], this->joint_trajectory[mode-3].back()[11]
      );
      commands.dqJ_cmd = Vector12d::Zero();
      // now apply new command
      commands.qJ_cmd.segment(3, 3) = Eigen::Vector3d(  this->joint_trajectory[mode][case_index][0],
                                                        this->joint_trajectory[mode][case_index][1],
                                                        this->joint_trajectory[mode][case_index][2]);
                                                        
      commands.dqJ_cmd.segment(3, 3) = Eigen::Vector3d( this->joint_velocity_trajectory[3][case_index][0],
                                                        this->joint_velocity_trajectory[3][case_index][1],
                                                        this->joint_velocity_trajectory[3][case_index][2]);
      RCLCPP_DEBUG(get_node()->get_logger(), "Moving right front leg to position: %f, %f, %f", 
                                               commands.qJ_cmd[3], commands.qJ_cmd[4], commands.qJ_cmd[5]);
      return controller_interface::return_type::OK;
      break;
    }

    case 7: // wait for some seconds before the next pushing
    {
      int case_index = this->cur_index - this->count_accummulation[mode - 1];
      RCLCPP_INFO(get_node()->get_logger(), "the case index is %d", case_index);
      commands.qJ_cmd = Eigen::Matrix<double, 12, 1>(
        this->joint_trajectory[mode-4].back()[0], this->joint_trajectory[mode-4].back()[1],
        this->joint_trajectory[mode-4].back()[2], this->joint_trajectory[mode-4].back()[3],
        this->joint_trajectory[mode-4].back()[4], this->joint_trajectory[mode-4].back()[5],
        this->joint_trajectory[mode-4].back()[6], this->joint_trajectory[mode-4].back()[7],
        this->joint_trajectory[mode-4].back()[8], this->joint_trajectory[mode-4].back()[9],
        this->joint_trajectory[mode-4].back()[10], this->joint_trajectory[mode-4].back()[11]
      );
      commands.dqJ_cmd = Vector12d::Zero();
      RCLCPP_DEBUG(get_node()->get_logger(), "Waiting for some seconds");
      return controller_interface::return_type::OK;
      break;
    }

    default:
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Unhandled mode: %zu", mode);
      return controller_interface::return_type::ERROR;
      break;
    }
  }
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