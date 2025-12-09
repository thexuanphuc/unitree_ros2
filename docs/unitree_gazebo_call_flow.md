# Unitree Gazebo System — Function Call Flow

This document summarizes the key functions in `unitree_gazebo/src/unitree_gazebo_system.cpp` that form the bridge between ROS 2 controllers and the Gazebo simulation. The table lists the typical runtime and initialization call order, who calls each function, when it is called, and what it does.

| Order | Function (signature) | Called by | When / Trigger | Purpose / Effect | Next / Calls |
|---:|---|---|---|---|---|
| 1 | `UnitreeGazeboSystem::on_init(const hardware_interface::HardwareInfo &)` | ros2_control lifecycle / system loader | During hardware system construction / registration | Performs SystemInterface initialization checks. Returns SUCCESS or ERROR. | (returns to caller) |
| 2 | `UnitreeGazeboSystem::initSim(rclcpp::Node::SharedPtr & model_nh, gazebo::physics::ModelPtr parent_model, const hardware_interface::HardwareInfo & hardware_info, sdf::ElementPtr sdf)` | `gazebo_ros2_control` / Gazebo plugin initializer | When Gazebo loads the plugin / simulation starts | Top-level simulator initialization: stores model/node pointers, checks physics, and invokes joint & sensor registration. | Calls `registerJoints(...)` and `registerSensors(...)` |
| 3 | `UnitreeGazeboSystem::registerJoints(const hardware_interface::HardwareInfo &, gazebo::physics::ModelPtr)` | Called from `initSim` | During plugin initialization | Finds gazebo joints matching URDF/hardware_info, allocates state/command interface entries, sets initial positions/velocities/efforts, registers mimics and gains (`Kp`, `Kd`). | Populates `state_interfaces_` and `command_interfaces_` used by ros2_control |
| 4 | `UnitreeGazeboSystem::registerSensors(const hardware_interface::HardwareInfo &, gazebo::physics::ModelPtr)` | Called from `initSim` | During plugin initialization | Finds IMU and force-torque sensors, casts to Gazebo sensor types, and registers sensor state interfaces pointing into internal sensor data arrays. | Populates `state_interfaces_` for sensors |
| 5 | `UnitreeGazeboSystem::export_state_interfaces()` | ros2_control Resource Manager | When ros2_control requests state interfaces | Returns the vector of `StateInterface` objects (moves ownership). These supply controller reads. | ros2_control exposes these interfaces to controllers |
| 6 | `UnitreeGazeboSystem::export_command_interfaces()` | ros2_control Resource Manager | When ros2_control requests command interfaces | Returns the vector of `CommandInterface` objects (moves ownership). Controllers write to these. | Controllers write to these command handles during `update()` cycles |
| 7 | `UnitreeGazeboSystem::perform_command_mode_switch(const std::vector<std::string> & start, const std::vector<std::string> & stop)` | Controller manager / ros2_control | When controllers start/stop or switch resources | Updates internal `joint_control_methods_` bitmask per joint to enable/disable POSITION/VELOCITY/EFFORT control. | Affects subsequent behavior in `write()` |
| 8 | `UnitreeGazeboSystem::on_activate(const rclcpp_lifecycle::State &)` | Lifecycle manager / ros2_control | On hardware lifecycle activation | Lifecycle transition hook; here returns SUCCESS (no extra action). | (ready to run) |
| 9 | `UnitreeGazeboSystem::read(const rclcpp::Time & time, const rclcpp::Duration & period)` | ros2_control hardware update loop (read phase) | Called every controller cycle before controller `update()` | Reads Gazebo joint positions/velocities/forces and sensor values into `joint_position_`, `joint_velocity_`, `joint_effort_`, `imu_sensor_data_`, `ft_sensor_data_`. These back the exported `StateInterface`s. | Controllers read state and compute commands |
| 10 | Controller computation (outside this file) — controller manager calls controllers' `update()` | Controller manager | Between `read()` and `write()` each cycle | Controllers read state interfaces and write desired values into the exported command interfaces (position/velocity/effort/Kp/Kd). | Data is stored in the pointers owned by this system (`joint_*_cmd_`, `Kp_cmd_`, `Kd_cmd_`) |
| 11 | `UnitreeGazeboSystem::write(const rclcpp::Time & time, const rclcpp::Duration & period)` | ros2_control hardware update loop (write phase) | Called every controller cycle after controller `update()` | Applies the controller-written command values to Gazebo joints.
|   |  |  |  | - Updates mimic joints (copies/multiplies commands from mimicked joints)
|   |  |  |  | - For each joint, if POSITION enabled -> `sim_joint->SetPosition(0, position_cmd, true)`
|   |  |  |  | - If VELOCITY enabled -> `sim_joint->SetVelocity(0, velocity_cmd)`
|   |  |  |  | - If EFFORT enabled -> compute effort = effort_cmd + Kp*(pos_error) + Kd*(vel_error); call `sim_joint->SetForce(0, effort)` |
| 12 | `UnitreeGazeboSystem::on_deactivate(const rclcpp_lifecycle::State &)` | Lifecycle manager / ros2_control | On hardware lifecycle deactivation | Lifecycle transition hook; here returns SUCCESS (no extra action). | (hardware inactive) |

Notes and important details:

- The high-level control loop order (per cycle) is: `read(time, period)` -> controllers compute and write to command interfaces -> `write(time, period)`.
- `export_state_interfaces()` and `export_command_interfaces()` hand pointers into this system's internal storage; controllers interact only with those interfaces.
- `perform_command_mode_switch(...)` toggles which control modes are active per-joint; `write()` checks `joint_control_methods_` bits before applying commands.
- Mimic joints are updated at the top of `write()` so their command values move consistently with their mimicked joint.
- Sensor reading mapping: IMU and FT sensor values are stored in `imu_sensor_data_` and `ft_sensor_data_` arrays and exposed via `state_interfaces_` entries created in `registerSensors()`.

Example control flow (runtime):

1. Gazebo plugin loads -> `initSim(...)` called -> `registerJoints(...)`, `registerSensors(...)` populate interfaces.
2. ros2_control requests `export_state_interfaces()` and `export_command_interfaces()` and exposes them to controllers.
3. Controller Manager starts controllers -> `perform_command_mode_switch(...)` updates mode bits.
4. Each control cycle: `read(...)` -> controllers read and compute -> controllers write to command interfaces -> `write(...)` applies to Gazebo.

If you want, I can also:
- Add a visual sequence diagram (PlantUML) for this flow.
- Insert this Markdown file into a different path or update README to link to it.

---
Generated from `unitree_gazebo_system.cpp` (analysis of initialization and runtime calls).