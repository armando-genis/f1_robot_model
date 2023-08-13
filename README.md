# f1_robot_model
Car simulator f1 foxy tenth in ROS2

You must install: 
sudo apt install ros-<ros2-distro>-joint-state-publisher-gui
sudo apt install ros-<ros2-distro>-xacro
sudo apt install ros-<ros2-distro>-gazebo-ros-pkgs

Write in shell #1
cd ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon build --packages-select f1_robot_model
ros2 launch f1_robot_model display.launch.py

Write in shell #2
ros2 topic pub /racecar/cmd_demo geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 1.0, z: 1.0}, angular: {x: 0.0, y: 0.0, z: 5.0}}'
