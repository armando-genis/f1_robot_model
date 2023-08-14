# f1_robot_model

Car simulator f1 foxy tenth in ROS2

## You must install: 

```bash
sudo apt install ros-<ros2-distro>-joint-state-publisher-gui
sudo apt install ros-<ros2-distro>-xacro
sudo apt install ros-<ros2-distro>-gazebo-ros-pkgs
sudo apt install ros-<ros2-distro>-ackermann-msgs
```

## Run Locally

Write in shell #1

```bash
cd ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon build --packages-select f1_robot_model
ros2 launch f1_robot_model display.launch.py
```

Write in shell #2
```bash

ros2 topic pub /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, drive: {speed: 0.0, steering_angle: 0.5}}"
```
## Distribution
```bash
f1_robot_model/
|-- src/
|   |-- ackermann_to_cmd_vel.cpp
|-- world/
|   |-- empty_world.sdf
|   |-- my_world.sdf
|-- urdf/
|   |-- macros.xacro
|   |-- materials.xacro
|   |-- racecar.gazebo
|   |-- racecar.urdf
|-- rviz/
|   |-- urdf_config.rviz
|-- launch/
|   |-- display.launch.py
|-- CMakeLists.txt
|-- package.xml
```

## Authors

- [@armando-genis](https://github.com/armando-genis)