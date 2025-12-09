// we have mujoco_system_interface(work with ros2), and unitree_mujoco_system (bridge between A1_shared_data andmujoco_system_interface mujoco_system_interface)

#ifndef UNITREE_MUJOCO__UNITREE_MUJOCO_SYSTEM_HPP_
#define UNITREE_MUJOCO__UNITREE_MUJOCO_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "std_msgs/msg/bool.hpp"
#include "mujoco_system_interface.hpp"

namespace unitree_mujoco
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Forward declaration
class UnitreeMujocoSystemPrivate;

// we are trying to simplify and mimic the gazebo_ros2_control::GazeboSystemInterface
// simulated `ros2_control` `hardware_interface::SystemInterface`.
// so we make the UnitreeMujocoSystem as self contained as possible, remove all the controller auto matching stuff

class UnitreeMujocoSystem : public MujocoSystemInterface
{
public:
  // Documentation Inherited
  CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) override;

  // Documentation Inherited
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Documentation Inherited
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Documentation Inherited
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // Documentation Inherited
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Documentation Inherited
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // Documentation Inherited
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // Documentation Inherited
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // Documentation Inherited
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    const hardware_interface::HardwareInfo & hardware_info) override;

private:
  void registerJoints(
    const hardware_interface::HardwareInfo & hardware_info);

  void registerSensors(
    const hardware_interface::HardwareInfo & hardware_info);

  /// \brief Private data class
  std::unique_ptr<UnitreeMujocoSystemPrivate> dataPtr; // TODO: should we use directly low level command here? or should we do a sarated class for it?
};

} // namespace unitree_mujoco

#endif // UNITREE_MUJOCO__UNITREE_MUJOCO_SYSTEM_HPP_