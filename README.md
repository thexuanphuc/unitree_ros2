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
' > selected_fields.txt

```