#ifndef UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_
#define UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_

#include "unitree_controller/unitree_controller_interface.hpp"
#include "unitree_controller/types.hpp"
#include "unitree_controller/visibility_control.h"
#include "unitree_controller/pd_controller.hpp"

#include "unitree_msgs/srv/set_control_mode.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "rclcpp/rclcpp.hpp"

// #include <xtensor/xarray.hpp>
// #include <xtensor/xio.hpp>
// #include <xtensor/xnpy.hpp>

namespace unitree_controller
{

using namespace std::chrono_literals;  // NOLINT

class UnitreeController : public UnitreeControllerInterface
{
public:
  UNITREE_CONTROLLER_PUBLIC 
  UnitreeController();

private:

  std::vector<std::vector<double>> joint_trajectory_;
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
  int control_mode_phuc_count = 0;
  // Services
  rclcpp::Service<unitree_msgs::srv::SetControlMode>::SharedPtr set_contro_mode_srv_;
  realtime_tools::RealtimeBuffer<ControlMode> control_mode_rt_buffer_;
  void setControlModeCallback(const std::shared_ptr<unitree_msgs::srv::SetControlMode::Request> request,
                              std::shared_ptr<unitree_msgs::srv::SetControlMode::Response> response);
};

} // namespace unitree_controller

#endif // UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_