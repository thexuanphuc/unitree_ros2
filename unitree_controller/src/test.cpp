#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>

class UnitreeController : public controller_interface::ControllerInterface
{
public:
  UnitreeController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {
      "FR_hip_joint/position",
      "FR_thigh_joint/position",
      "FR_calf_joint/position"
    };
    return config;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {
      "FR_hip_joint/position",
      "FR_thigh_joint/position",
      "FR_calf_joint/position",
      "FR_hip_joint/velocity",
      "FR_thigh_joint/velocity",
      "FR_calf_joint/velocity"
    };
    return config;
  }

  controller_interface::return_type init(const std::string & controller_name) override
  {
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) {
      return ret;
    }

    // Load trajectory from JSON
    try {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("unitree_controller");
      std::string file_path = pkg_share + "/config/joint_trajectory.json";
      std::ifstream file(file_path);
      if (!file.is_open()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to open %s", file_path.c_str());
        return controller_interface::return_type::ERROR;
      }
      nlohmann::json json_data;
      file >> json_data;
      for (const auto & row : json_data) {
        std::vector<double> joints;
        for (const auto & value : row) {
          joints.push_back(value.get<double>());
        }
        if (joints.size() != 3) {
          RCLCPP_ERROR(get_node()->get_logger(), "Invalid row size in trajectory");
          return controller_interface::return_type::ERROR;
        }
        joint_trajectory_.push_back(joints);
      }
      RCLCPP_INFO(get_node()->get_logger(), "Loaded trajectory with %zu points", joint_trajectory_.size());
      if (joint_trajectory_.size() != 101) {
        RCLCPP_ERROR(get_node()->get_logger(), "Expected 101 points, got %zu", joint_trajectory_.size());
        return controller_interface::return_type::ERROR;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to load trajectory: %s", e.what());
      return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    command_interfaces_.resize(3, nullptr);
    state_interfaces_.resize(6, nullptr);
    trajectory_index_ = 0;
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      if (!command_interfaces_[i]) {
        RCLCPP_ERROR(get_node()->get_logger(), "Command interface %zu not assigned", i);
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (joint_trajectory_.empty()) {
      RCLCPP_WARN_ONCE(get_node()->get_logger(), "No trajectory loaded");
      return controller_interface::return_type::OK;
    }

    // Send joint position commands
    const auto & joints = joint_trajectory_[trajectory_index_];
    for (size_t i = 0; i < 3; ++i) {
      if (command_interfaces_[i]) {
        command_interfaces_[i]->set_value(joints[i]);
      }
    }

    // Increment trajectory index (loop every 101 points)
    trajectory_index_ = (trajectory_index_ + 1) % joint_trajectory_.size();

    return controller_interface::return_type::OK;
  }

private:
  std::vector<std::vector<double>> joint_trajectory_;
  size_t trajectory_index_;
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces_;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(UnitreeController, controller_interface::ControllerInterface)
