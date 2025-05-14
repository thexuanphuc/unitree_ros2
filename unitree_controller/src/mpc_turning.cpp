#include "unitree_controller/mpc_turning.hpp"

namespace unitree_controller
{

MpcStartTurning::MpcStartTurning(const Vector12d& qJ, const Vector12d& dqJ, const Vector12d& tauJ, 
                           const Vector12d& Kp, const Vector12d& Kd)
  : qJ_cmd_(qJ), dqJ_cmd_(dqJ), tauJ_cmd_(tauJ), Kp_cmd_(Kp), Kd_cmd_(Kd)
{
  if (Kp.minCoeff() < 0.0) {
    throw std::invalid_argument("[MpcStartTurning] Kp.minCoeff() must be non-negative");
  }
  if (Kd.minCoeff() < 0.0) {
    throw std::invalid_argument("[MpcStartTurning] Kd.min_coeff() must be non-negative");
  }
}

std::tuple<Vector12d, Vector12d, Vector12d, Vector12d, Vector12d, int> 
MpcStartTurning::compute_desired_trajectory()
{
  control_mode_phuc_count_ += 1;

  Vector12d qJ_cmd = Vector12d::Zero();
  Vector12d dqJ_cmd = Vector12d::Zero();
  Vector12d tauJ_cmd = Vector12d::Zero();
  Vector12d Kp_cmd = Vector12d::Constant(90.0);
  Vector12d Kd_cmd = Vector12d::Constant(16.0);
  // Set the command to zero torque initially , TODO dont do this
  if(control_mode_phuc_count_ < 0) {
    Vector12d qJ_cmd = Vector12d::Constant(this->PosStop_custom);
    Vector12d dqJ_cmd = Vector12d::Constant(this->VelStopF_custom);

    // return std::make_tuple(std::move(qJ_cmd), std::move(dqJ_cmd), std::move(tauJ_cmd), 
    //                        std::move(Kp_cmd), std::move(Kd_cmd), -1);
  }

  cur_index_ = static_cast<int>(floor(control_mode_phuc_count_ / iteration_tracking_));

  size_t mode = 0;
  while (mode < count_accummulation_.size() && 
         cur_index_ >= count_accummulation_[mode]) {
    ++mode;
  }

  if (control_mode_phuc_count_ >= max_count) {
    // reset the count to 0
    control_mode_phuc_count_ = 0;
    cur_index_ = static_cast<int>(floor(control_mode_phuc_count_ / iteration_tracking_));
  }


  switch (mode) {
    case 0: // Mode 0: Normal sitting on board
    {
      qJ_cmd = Eigen::Matrix<double, 12, 1>(
        this->joint_trajectory_[mode][0][0], this->joint_trajectory_[mode][1][0],
        this->joint_trajectory_[mode][2][0], this->joint_trajectory_[mode][3][0],
        this->joint_trajectory_[mode][4][0], this->joint_trajectory_[mode][5][0],
        this->joint_trajectory_[mode][6][0], this->joint_trajectory_[mode][7][0],
        this->joint_trajectory_[mode][8][0], this->joint_trajectory_[mode][9][0],
        this->joint_trajectory_[mode][10][0], this->joint_trajectory_[mode][11][0]
      );
      dqJ_cmd = Vector12d::Zero();
      break;
    }

    case 1: // Mode 1: Move body to the right
    {
      int case_index = cur_index_ - count_accummulation_[mode-1];
      qJ_cmd = Eigen::Matrix<double, 12, 1>(
        joint_trajectory_[mode][case_index][0], joint_trajectory_[mode][case_index][1],
        joint_trajectory_[mode][case_index][2], joint_trajectory_[mode][case_index][3],
        joint_trajectory_[mode][case_index][4], joint_trajectory_[mode][case_index][5],
        joint_trajectory_[mode][case_index][6], joint_trajectory_[mode][case_index][7],
        joint_trajectory_[mode][case_index][8], joint_trajectory_[mode][case_index][9],
        joint_trajectory_[mode][case_index][10], joint_trajectory_[mode][case_index][11]
      );
      dqJ_cmd = Vector12d::Zero();
      break;
    }

    case 2: // Mode 2: wait for some seconds
    {
      qJ_cmd = Eigen::Matrix<double, 12, 1>(
        joint_trajectory_[mode-1].back()[0], joint_trajectory_[mode-1].back()[1],
        joint_trajectory_[mode-1].back()[2], joint_trajectory_[mode-1].back()[3],
        joint_trajectory_[mode-1].back()[4], joint_trajectory_[mode-1].back()[5],
        joint_trajectory_[mode-1].back()[6], joint_trajectory_[mode-1].back()[7],
        joint_trajectory_[mode-1].back()[8], joint_trajectory_[mode-1].back()[9],
        joint_trajectory_[mode-1].back()[10], joint_trajectory_[mode-1].back()[11]
      );
      dqJ_cmd = Vector12d::Zero();
      break;
    }

    case 3: // Mode 3: Move body to the left
    {
      int case_index = cur_index_ - count_accummulation_[mode-1];
      qJ_cmd = Eigen::Matrix<double, 12, 1>(
        joint_trajectory_[2][case_index][0], joint_trajectory_[2][case_index][1],
        joint_trajectory_[2][case_index][2], joint_trajectory_[2][case_index][3],
        joint_trajectory_[2][case_index][4], joint_trajectory_[2][case_index][5],
        joint_trajectory_[2][case_index][6], joint_trajectory_[2][case_index][7],
        joint_trajectory_[2][case_index][8], joint_trajectory_[2][case_index][9],
        joint_trajectory_[2][case_index][10], joint_trajectory_[2][case_index][11]
      );
      dqJ_cmd = Vector12d::Zero();
      break;
    }

    case 4: // Mode 4: Move body to the center
    {
      int case_index = cur_index_ - count_accummulation_[mode-1];
      qJ_cmd = Eigen::Matrix<double, 12, 1>(
        joint_trajectory_[3][case_index][0], joint_trajectory_[3][case_index][1],
        joint_trajectory_[3][case_index][2], joint_trajectory_[3][case_index][3],
        joint_trajectory_[3][case_index][4], joint_trajectory_[3][case_index][5],
        joint_trajectory_[3][case_index][6], joint_trajectory_[3][case_index][7],
        joint_trajectory_[3][case_index][8], joint_trajectory_[3][case_index][9],
        joint_trajectory_[3][case_index][10], joint_trajectory_[3][case_index][11]
      );
      dqJ_cmd = Vector12d::Zero();
      break;
    }

    case 5: // Mode 5: just sit down at the center for 7.5 seconds
    {
      qJ_cmd = Eigen::Matrix<double, 12, 1>(
        this->joint_trajectory_[mode][0][0], this->joint_trajectory_[mode][1][0],
        this->joint_trajectory_[mode][2][0], this->joint_trajectory_[mode][3][0],
        this->joint_trajectory_[mode][4][0], this->joint_trajectory_[mode][5][0],
        this->joint_trajectory_[mode][6][0], this->joint_trajectory_[mode][7][0],
        this->joint_trajectory_[mode][8][0], this->joint_trajectory_[mode][9][0],
        this->joint_trajectory_[mode][10][0], this->joint_trajectory_[mode][11][0]
      );
      dqJ_cmd = Vector12d::Zero();
      break;
    }

    default: // waiting mode
    {
      break;
    }
  }


  return std::make_tuple(std::move(qJ_cmd), std::move(dqJ_cmd), std::move(tauJ_cmd), 
                         std::move(Kp_cmd), std::move(Kd_cmd), static_cast<int>(mode));
}

bool MpcStartTurning::load_config()
{
  all_trajectory_length_ = 0;

  // Load position trajectory
  for (size_t i = 0; i < config_pos_files_.size(); ++i) {
    try {
      std::ifstream file(config_pos_files_[i].path);
      if (!file.is_open()) {
        success = false;
        continue;
      }

      nlohmann::json json_data;
      file >> json_data;
      file.close();
      for (const auto& row : json_data) {
        std::vector<double> joints;
        for (const auto& value : row) {
          joints.push_back(value.get<double>());
        }
        joint_trajectory_[i].push_back(joints);
      }
    } catch (const std::exception& e) {
      success = false;
    }
  }

  count_accummulation_.push_back(2000 * 1); // 1 state for sitting
  count_accummulation_.push_back(count_accummulation_.back() + joint_trajectory_[1].size()); // 1 move to the right
  count_accummulation_.push_back(count_accummulation_.back() + 1000); // 2.5 seconds wait
  count_accummulation_.push_back(count_accummulation_.back() + joint_trajectory_[2].size()); // 1 move to the left
  count_accummulation_.push_back(count_accummulation_.back() + joint_trajectory_[3].size()); // 1 move to the center
  count_accummulation_.push_back(count_accummulation_.back() + 3000); // 7.5 seconds wait
  max_count = count_accummulation_.back() * iteration_tracking_;

  return true;
}

} // namespace unitree_controller