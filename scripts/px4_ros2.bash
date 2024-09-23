# first you need to cd to PX4-Autopilot directory

source /opt/ros/foxy/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash 
source /usr/share/colcon_cd/function/colcon_cd.sh

export PYTHONOPTIMIZE=1
export ROS_DOMAIN_ID=0
# source install/setup.bash

source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

# expose px4 commands to shell
 export PATH=$PATH:$(pwd)/build/px4_sitl_default/bin

# custom gazebo world
# export PX4_SITL_WORLD=~/ros2_ws/src/worlds/interaction.world
