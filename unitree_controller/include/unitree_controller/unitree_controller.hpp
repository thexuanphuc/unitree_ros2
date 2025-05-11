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
  int control_mode_phuc_count = 100;

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

  std::vector<ConfigFile> config_files_ = {
    {"pushing_pos", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/pushing.json"},
    {"pushing_vel", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/pushing_vel.json"},
    {"lifting_pos", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/lifting.json"},
    {"lifting_vel", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/lifting_vel.json"},
    {"sitting_on_board", ament_index_cpp::get_package_share_directory("unitree_controller") + "/config/onboard_legs.json"}
  };

  // Trajectory data
  std::array<std::vector<std::vector<double>>, 5> joint_trajectory; 
  bool is_lifting = false;
  bool loadConfigFiles();
};

} // namespace unitree_controller

#endif // UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_