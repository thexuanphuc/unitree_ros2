#ifndef UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_
#define UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_

#include "unitree_controller/unitree_controller_interface.hpp"
#include "unitree_controller/types.hpp"
#include "unitree_controller/visibility_control.h"
#include "unitree_controller/pd_controller.hpp"

#include "unitree_msgs/srv/set_control_mode.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>
#include <nlohmann/json.hpp>  // Correct include
#include <string>
#include <cmath>

namespace unitree_controller
{
using namespace std::chrono_literals;  // NOLINT

class UnitreeController : public UnitreeControllerInterface
{
public:
  UNITREE_CONTROLLER_PUBLIC 
  UnitreeController();

private:

  void declare_parameters() override;

  controller_interface::CallbackReturn read_parameters() override;

  std::vector<std::string> get_joint_names() const override;

  std::vector<std::string> get_sensor_names() const override;

  controller_interface::return_type update(
  const rclcpp::Time & time, const rclcpp::Duration & period,
  const UnitreeStates & states, UnitreeCommands & commands) override;

  // interfaces
  std::vector<std::string> joint_names_, sensor_names_;

  // node parameters
  double control_rate_, control_period_;

  // Runtime controllers 
  ControlMode control_mode_;
  PDController zero_torque_controller_, standing_up_controller_, sitting_down_controller_;
  // use 5 iterations to track the trajectory
  int iteration_tracking = 3;
  int all_trajectory_length = 0;
  int max_count = 0;
  int control_mode_phuc_count = -000;
  int push_times = 4;
  int cur_index = 0;
  // moving mode 
  // 0 is just normal sitting on board
  // 1 is move the body to the right (move 4 legs)           
  // 2 is move the left front leg into center
  // 3 is move the body back to the left
  // 4 is move the right leg to the floor
  // 5 is push the front right leg on the floor N times
  // 6 is move the front right leg back to the skateboard
  // 7 just wait for some seconds before the next pushing
  std::vector<int> count_accummulation;

  // Services
  rclcpp::Service<unitree_msgs::srv::SetControlMode>::SharedPtr set_contro_mode_srv_;
  realtime_tools::RealtimeBuffer<ControlMode> control_mode_rt_buffer_;
  void setControlModeCallback(const std::shared_ptr<unitree_msgs::srv::SetControlMode::Request> request,
                              std::shared_ptr<unitree_msgs::srv::SetControlMode::Response> response);

  // to load the joint position
  struct ConfigFile {
    std::string name;
    std::string path;
  };

  std::vector<ConfigFile> config_pos_files_ = {
    {"sitting_on_board", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/onboard_sitting.json"},
    {"trajectory_movebody_1", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/trajectory_movebody_1.json"},
    {"trajectory_moveleg", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/trajectory_moveleg.json"},
    {"trajectory_movebody_2", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/trajectory_movebody_2.json"},
    {"lifting_down", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/lifting_down.json"},
    {"pushing_pos", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/pushing.json"},
    {"lifting_up", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/lifting_up.json"}
  };

  std::vector<ConfigFile> config_vel_files_ = {
    {"trajectory_moveleg_vel", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/trajectory_moveleg_vel.json"},
    {"lifting_down_vel", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/lifting_down_vel.json"},
    {"pushing_vel", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/pushing_vel.json"},
    {"lifting_up_vel", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/lifting_up_vel.json"}
  };

  // Trajectory data
  std::array<std::vector<std::vector<double>>, 7> joint_trajectory; 
  std::array<std::vector<std::vector<double>>, 4> joint_velocity_trajectory;
  bool is_lifting = false;
  bool loadConfigFiles();

};

} // namespace unitree_controller

#endif // UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_