#ifndef UNITREE_CONTROLLER_MPC_START_PUSH_HPP_
#define UNITREE_CONTROLLER_MPC_START_PUSH_HPP_

#include <string>
#include <vector>
#include <array>
#include <tuple>
#include <fstream>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "nlohmann/json.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace unitree_controller
{
using Vector12d = Eigen::Matrix<double, 12, 1>;

class MpcStartPush
{
public:
  // Constructor
  MpcStartPush(const Vector12d& qJ = Vector12d::Zero(),
               const Vector12d& dqJ = Vector12d::Zero(),
               const Vector12d& tauJ = Vector12d::Zero(),
               const Vector12d& Kp = Vector12d::Zero(),
               const Vector12d& Kd = Vector12d::Zero());

  // Static factory method for zero initialization TODO check this
  static MpcStartPush Zero() {
    return MpcStartPush();
  }

  // Compute desired trajectory based on mode and index
  std::tuple<Vector12d, Vector12d, Vector12d, Vector12d, Vector12d> compute_desired_trajectory(
    const size_t mode, const int cur_index);

  // Load configuration files
  bool load_config();

  // Getters
  const Vector12d& qJ_cmd() const { return qJ_cmd_; }
  const Vector12d& dqJ_cmd() const { return dqJ_cmd_; }
  const Vector12d& tauJ_cmd() const { return tauJ_cmd_; }
  const Vector12d& Kp_cmd() const { return Kp_cmd_; }
  const Vector12d& Kd_cmd() const { return Kd_cmd_; }
  int get_iteration_tracking() const { return iteration_tracking_; }
  int get_max_count() const { return max_count; }
  const std::vector<int>& get_count_accumulation() const { return count_accummulation_; }

private:
  // Command variables
  Vector12d qJ_cmd_;
  Vector12d dqJ_cmd_;
  Vector12d tauJ_cmd_;
  Vector12d Kp_cmd_;
  Vector12d Kd_cmd_;

  // Trajectory tracking parameters
  int iteration_tracking_ = 3;
  int all_trajectory_length_ = 0;
  int max_count = 0;
  int push_times_ = 4;
  int cur_index_ = 0;
  std::vector<int> count_accummulation_;

  // Configuration files
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
  std::array<std::vector<std::vector<double>>, 7> joint_trajectory_;
  std::array<std::vector<std::vector<double>>, 4> joint_velocity_trajectory_;
  bool is_lifting_ = false;
};

} // namespace unitree_controller

#endif // UNITREE_CONTROLLER_MPC_START_PUSH_HPP_