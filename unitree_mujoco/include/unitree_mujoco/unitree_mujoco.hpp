#ifndef UNITREE_HARDWARE__UNITREE_HARDWARE_HPP_
#define UNITREE_HARDWARE__UNITREE_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>

#include "rclcpp/macros.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "unitree_mujoco/mujoco_interface_type_values.hpp"
#include "unitree_mujoco/visibility_control.h"
#include <a1_data_shared.h>

// TODO: check the correspondance with mujoco
namespace LEG_ORDER{
  constexpr int FR_ = 0;       // leg index
  constexpr int FL_ = 1;
  constexpr int RR_ = 2;
  constexpr int RL_ = 3;

  constexpr int FR_0 = 0;      // joint index
  constexpr int FR_1 = 1;      
  constexpr int FR_2 = 2;

  constexpr int FL_0 = 3;
  constexpr int FL_1 = 4;
  constexpr int FL_2 = 5;

  constexpr int RR_0 = 6;
  constexpr int RR_1 = 7;
  constexpr int RR_2 = 8;

  constexpr int RL_0 = 9;
  constexpr int RL_1 = 10;
  constexpr int RL_2 = 11;
}

namespace unitree_mujoco
{
class UnitreeMujoco final : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UnitreeMujoco)

  UNITREE_MUJOCO_PUBLIC
  UnitreeMujoco();

  UNITREE_MUJOCO_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  UNITREE_MUJOCO_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  UNITREE_MUJOCO_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  UNITREE_MUJOCO_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_MUJOCO_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_MUJOCO_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  UNITREE_MUJOCO_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  std::vector<double> qJ_, dqJ_, tauJ_, 
                      imu_quaternion_, imu_gyroscope_, imu_accelerometer_, foot_force_sensor_;
  std::vector<double> qJ_cmd_, dqJ_cmd_, tauJ_cmd_, Kp_cmd_, Kd_cmd_;

  static constexpr std::array<int, 12> joints_ = {LEG_ORDER::FL_0, 
                                                  LEG_ORDER::FL_1, 
                                                  LEG_ORDER::FL_2,
                                                  LEG_ORDER::FR_0, 
                                                  LEG_ORDER::FR_1, 
                                                  LEG_ORDER::FR_2, 
                                                  LEG_ORDER::RL_0, 
                                                  LEG_ORDER::RL_1, 
                                                  LEG_ORDER::RL_2, 
                                                  LEG_ORDER::RR_0, 
                                                  LEG_ORDER::RR_1, 
                                                  LEG_ORDER::RR_2};

  static constexpr std::array<int, 4> feet_ = {LEG_ORDER::FL_,
                                               LEG_ORDER::FR_,
                                               LEG_ORDER::RL_,
                                               LEG_ORDER::RR_};

  a1_shm::SharedDataA1* shm_;
  uint64_t seq0_, seq1_;
  bool data_consistent = false;

};

}  // namespace unitree_mujoco


#endif  // UNITREE_HARDWARE__UNITREE_HARDWARE_HPP_