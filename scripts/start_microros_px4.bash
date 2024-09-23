# first you need to cd to microros_ws directory
source /opt/ros/foxy/setup.bash
export PYTHONOPTIMIZE=1
export ROS_DOMAIN_ID=0
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 ROS_DOMAIN_ID=0
