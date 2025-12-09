# unitree_ros2
An unofficial ROS 2 package for Unitree's quadruped robots.

Tested on **Ubuntu 22.04 LTS** with **ROS 2 Humble**.

## Installation

### Local Installation

Install the required dependencies by running:

```bash
bash unitree_controller/script/install_requirement.sh
```

### Docker

#### Build the Docker Image

```bash
docker build . -t unitree_ros2:latest
```

#### Run the Docker Container

```bash
docker compose up a1
```

### Notes

- TODO: check communication between the Docker container and the real robot.



## echo the state 

```
 ros2 topic echo /joint_states | awk '
  /^header:/ { getline; getline; print; getline; print }
  /^position:/ {
    print
    for(i=1; i<=12; i++) {getline; print }
  }
  /^effort:/ {
    print
    for(i=1; i<=12; i++) {getline; print }
  }
' > position_output.txt

```



## echo the foot force 

```
ros2 topic echo /dynamic_joint_states | awk '
/sec:/ {print "sec: " $2}
/nanosec:/ {print "nanosec: " $2}
/- interface_names:/ {in_names=1; names=""; next}
in_names && /^  - / {names = names $2 " "; next}
in_names && !/^  - / {in_names=0}
/^  values:/ {in_values=1; next}
in_values && /^  - / {
    if (names == "force.z ") {
        print "force.z: " $2
        in_values=0
    }
    next
}
' > force_output.txt
```

## build and run

  colcon build --packages-ignore lcm gazebo_ros2_control_demos  --cmake-args -Wno-dev && source  install/setup.bash  && ros2 launch unitree_controller a1.launch.py


+ As we use the unitree mujoco, we just do 2 terminal here, as mujoco should run separatedly to avoid concurency.

## TODO: migration to mujoco:
no, right now i have the unitree_mujoco package. it was well done but only for other model, not for A1. 
now i want to take that as module (i has simulation, render etc)), then try to send data to that, the only thing is to rewrite the unitree_mujoco_bridge sdk1
