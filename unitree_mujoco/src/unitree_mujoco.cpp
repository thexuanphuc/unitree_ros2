#include "unitree_mujoco/unitree_mujoco.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace unitree_mujoco
{

UnitreeMujoco::UnitreeMujoco()
  : hardware_interface::SystemInterface(),
    // these comment is used to interact with ros2_control framework. we should keep it here, otherwise we need them for matching value,
    // otherwise we need to manually write the interference DDS/ or use ros2 message. TODO: should we remove the ros2_control, only use custom shared memory
    qJ_(), 
    dqJ_(), 
    tauJ_(), 
    imu_quaternion_(), 
    imu_gyroscope_(), 
    imu_accelerometer_(), 
    foot_force_sensor_(), 
    qJ_cmd_(), 
    dqJ_cmd_(), 
    tauJ_cmd_(), 
    Kp_cmd_(), 
    Kd_cmd_()
{
}


hardware_interface::CallbackReturn UnitreeMujoco::on_init(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("UnitreeMujoco"), "UnitreeMujoco::on_init() start ##############3");
  if ( 
  // this call to hardware_interface::SystemInterface::on_init(info)
  //  will auto create the info_ object as alias of info
  //  take and check info from robot_description/
    hardware_interface::SystemInterface::on_init(info) != 
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Joint states
  qJ_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  dqJ_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  tauJ_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // Imu states
  imu_quaternion_.resize(4, std::numeric_limits<double>::quiet_NaN());
  imu_gyroscope_.resize(3, std::numeric_limits<double>::quiet_NaN());
  imu_accelerometer_.resize(3, std::numeric_limits<double>::quiet_NaN());
  // Foot force sensor states 
  foot_force_sensor_.resize(4, std::numeric_limits<double>::quiet_NaN());
  // Joint commands
  qJ_cmd_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  dqJ_cmd_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  tauJ_cmd_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  Kp_cmd_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  Kd_cmd_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // check joint state and command interfaces
  if (info_.joints.size() != a1_shm::NJ)
  {
    RCLCPP_FATAL(rclcpp::get_logger("UnitreeMujoco"), "Number of joint is %zu. a1_shm::NJ expected.", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // check joint state interfaces
    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL( rclcpp::get_logger("UnitreeMujoco"), "Joint '%s'has %zu state interfaces. 3 expected.", joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (const auto & state_interface : joint.state_interfaces) 
    {
    if (!(state_interface.name == hardware_interface::HW_IF_POSITION ||
          state_interface.name == hardware_interface::HW_IF_VELOCITY ||
          state_interface.name == hardware_interface::HW_IF_EFFORT))
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("UnitreeMujoco"),
          "Joint '%s' has %s state interfaces. Expected %s, %s, or %s.", joint.name.c_str(),
          state_interface.name.c_str(), hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // check joint command interfaces
    if (joint.command_interfaces.size() != 5)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("UnitreeMujoco"),
        "Joint '%s' has %zu command interfaces. 5 expected.", joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (const auto & command_interface : joint.command_interfaces) 
    {
      if (!(command_interface.name == hardware_interface::HW_IF_POSITION ||
            command_interface.name == hardware_interface::HW_IF_VELOCITY ||
            command_interface.name == hardware_interface::HW_IF_EFFORT ||
            command_interface.name == HW_IF_POSITION_GAIN ||
            command_interface.name == HW_IF_VELOCITY_GAIN ))
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("UnitreeMujoco"),
          "Joint '%s' has %s command interface. Expected %s, %s, %s, %s, or %s.", joint.name.c_str(),
          command_interface.name.c_str(), hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT,
          HW_IF_POSITION_GAIN, HW_IF_VELOCITY_GAIN);

        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }
  
  // check Imu state interfaces
  if (info_.sensors[0].state_interfaces.size() != 10)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeMujoco"),
      "Sensor[0] (should be Imu) has %zu state interfaces. 10 expected.", info_.sensors[0].state_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[0].name == "orientation.x"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeMujoco"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[0]. Expected orientation.x",
      info_.sensors[0].state_interfaces[0].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[1].name == "orientation.y"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeMujoco"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[1]. Expected orientation.y",
      info_.sensors[0].state_interfaces[1].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[2].name == "orientation.z"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeMujoco"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[2]. Expected orientation.z",
      info_.sensors[0].state_interfaces[2].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[3].name == "orientation.w"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeMujoco"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[3]. Expected orientation.w",
      info_.sensors[0].state_interfaces[3].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!(info_.sensors[0].state_interfaces[4].name == "angular_velocity.x"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeMujoco"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[4]. Expected angular_velocity.x",
      info_.sensors[0].state_interfaces[4].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[5].name == "angular_velocity.y"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeMujoco"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[5]. Expected angular_velocity.y",
      info_.sensors[0].state_interfaces[5].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[6].name == "angular_velocity.z"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeMujoco"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[6]. Expected angular_velocity.z",
      info_.sensors[0].state_interfaces[6].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[7].name == "linear_acceleration.x"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeMujoco"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[7]. Expected linear_acceleration.x",
      info_.sensors[0].state_interfaces[7].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[8].name == "linear_acceleration.y"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeMujoco"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[8]. Expected linear_acceleration.y",
      info_.sensors[0].state_interfaces[8].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[9].name == "linear_acceleration.z"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeMujoco"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[9]. Expected linear_acceleration.z",
      info_.sensors[0].state_interfaces[9].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // check foot force sensor interfaces
  for (std::size_t i = 0; i < 4; i++)
  {
    if (info_.sensors[i+1].state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("UnitreeMujoco"),
        "Sensor[%zu] (should be foot force sensor) has %zu state interfaces. 1 expected.", i+1, info_.sensors[0].state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!(info_.sensors[i+1].state_interfaces[0].name == "force.z"))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("UnitreeMujoco"),
        "Sensor[%zu] (should be foot force sensor) has %s state interface. Expected force.z", 
        i+1, info_.sensors[i+1].state_interfaces[0].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("UnitreeMujoco"), "UnitreeMujoco::on_init() end, succefull pass all checking, data from xacro #################");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
UnitreeMujoco::export_state_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("UnitreeMujoco"), "UnitreeMujoco::export_state_interfaces start");
  // This function will take normal number and bind them to the state_interfaces, then send that state_interfaces into ROS2 high level
  // But where does the data come from?
  // The data comes from the this->udp_ object, which is a UDP connection to the robot, each time we call udp, assign value into these number

  std::vector<hardware_interface::StateInterface> state_interfaces;
  // Joint state
  for (std::size_t i = 0; i < a1_shm::NJ; i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &qJ_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &dqJ_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &tauJ_[i]));
  }
  // Imu state
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &imu_quaternion_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[1].name, &imu_quaternion_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[2].name, &imu_quaternion_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[3].name, &imu_quaternion_[3]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[4].name, &imu_gyroscope_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[5].name, &imu_gyroscope_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[6].name, &imu_gyroscope_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[7].name, &imu_accelerometer_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[8].name, &imu_accelerometer_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[9].name, &imu_accelerometer_[2]));
  // Foot force sensors 
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[1].name, info_.sensors[1].state_interfaces[0].name, &foot_force_sensor_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[2].name, info_.sensors[2].state_interfaces[0].name, &foot_force_sensor_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[3].name, info_.sensors[3].state_interfaces[0].name, &foot_force_sensor_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[4].name, info_.sensors[4].state_interfaces[0].name, &foot_force_sensor_[3]));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
UnitreeMujoco::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("UnitreeMujoco"), "UnitreeMujoco::export_command_interfaces start");
  // This function will take command from command_interfaces (high level) then assign to the normal number)
  // Then the normal number will be sent to the robot through this->udp_ object in write() function
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < a1_shm::NJ; i++){
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &qJ_cmd_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &dqJ_cmd_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &tauJ_cmd_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, HW_IF_POSITION_GAIN, &Kp_cmd_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, HW_IF_VELOCITY_GAIN, &Kd_cmd_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn UnitreeMujoco::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/) 
{
  RCLCPP_INFO(
    rclcpp::get_logger("UnitreeMujoco"), "Starting in UnitreeMujoco::on_activate() please wait ################");

  // Set some default values
  for (std::size_t i = 0; i < a1_shm::NJ; i++)
  {
    qJ_[i] = 0;
    dqJ_[i] = 0;
    tauJ_[i] = 0;
    qJ_cmd_[i] = 0;
    dqJ_cmd_[i] = 0;
    tauJ_cmd_[i] = 0;
    Kp_cmd_[i] = 0;
    Kd_cmd_[i] = 0;
  }

  for (std::size_t i = 0; i < 4; ++i) 
  {
    imu_quaternion_[i] = 0;
    if (i == 3) imu_quaternion_[i] = 1;
    foot_force_sensor_[i] = 0;
  }
  for (std::size_t i = 0; i < 3; ++i) 
  {
    imu_gyroscope_[i] = 0;
    imu_accelerometer_[i] = 0;
  }
  
  // tried to initialize the shared memory
  // Initialize Shared Memory
  shm_ = a1_shm::get();  // Store pointer in member variable
  if (shm_ == nullptr) {
        RCLCPP_DEBUG(rclcpp::get_logger("UnitreeMujoco"), "The shared memory is not initialized, run the mujoco simulation first");
        return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("UnitreeMujoco"), "Shared Memory Initialized from Ros2 Side ##############3");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnitreeMujoco::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/) 
{
  a1_shm::unlink_shared_memory();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type UnitreeMujoco::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  int read_counter = 0;
  while(1){    
    seq0_ = a1_shm::seq_load(shm_);
    if (seq0_ % 2 != 0) {
      continue; // Writer is busy
    }
    // Joint state
    std::memcpy(&this->qJ_[0]  , shm_->mt_st_q, a1_shm::NJ * sizeof(double));
    std::memcpy(&this->dqJ_[0] , shm_->mt_st_dq, a1_shm::NJ * sizeof(double));
    std::memcpy(&this->tauJ_[0], shm_->mt_st_tauEst, a1_shm::NJ * sizeof(double));
  
    // Imu state
    std::memcpy(&this->imu_quaternion_[0], shm_->imu_quaternion, 4 * sizeof(double));
    std::memcpy(&this->imu_gyroscope_[0], shm_->imu_gyroscope, 3 * sizeof(double));
    std::memcpy(&this->imu_accelerometer_[0], shm_->imu_accelerometer, 3 * sizeof(double));
    
    // foot sensor
    // TODO: check the order
    std::memcpy(&this->foot_force_sensor_[0], shm_->foot_force, 3 * sizeof(double));
    if (seq0_ == seq1_ && seq0_ % 2 == 0) {
      return hardware_interface::return_type::OK;
    }
    if(read_counter ++ > 5){
      RCLCPP_DEBUG(rclcpp::get_logger("UnitreeMujoco"), "Failed to read the state from shared memory, use old state instead");
    }
  }
}

hardware_interface::return_type UnitreeMujoco::write(
  const rclcpp::Time &  /* time */, const rclcpp::Duration & /* period */)
{


  for (int i = 0; i < a1_shm::NJ; ++i) {
    std::memcpy(&shm_->mt_cmd_q[i]   , &this->qJ_cmd_[i], sizeof(double));
    std::memcpy(&shm_->mt_cmd_dq[i]  , &this->dqJ_cmd_[i], sizeof(double));
    std::memcpy(&shm_->mt_cmd_tau[i] , &this->tauJ_cmd_[i], sizeof(double));
    std::memcpy(&shm_->mt_cmd_Kp[i]  , &this->Kp_cmd_[i], sizeof(double));
    std::memcpy(&shm_->mt_cmd_Kd[i]  , &this->Kd_cmd_[i], sizeof(double));
  }
  return hardware_interface::return_type::OK;
}

}  // namespace unitree_mujoco

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  unitree_mujoco::UnitreeMujoco,
  hardware_interface::SystemInterface)