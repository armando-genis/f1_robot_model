# f1_robot_model

'''
Created by Genis Armando <a01654262@mtec.mx> on august 2023.
'''


Car simulator f1 foxy tenth in ROS2

You must install: 
sudo apt install ros-<ros2-distro>-joint-state-publisher-gui
sudo apt install ros-<ros2-distro>-xacro
sudo apt install ros-<ros2-distro>-gazebo-ros-pkgs
sudo apt install ros-<ros2-distro>-ackermann-msgs


Write in shell #1
cd ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon build --packages-select f1_robot_model
ros2 launch f1_robot_model display.launch.py

Write in shell #2
ros2 topic pub /racecar/cmd_demo geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 1.0, z: 1.0}, angular: {x: 0.0, y: 0.0, z: 5.0}}'


ros2 topic pub /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, drive: {speed: 0.0, steering_angle: 0.5}}"
