# Uxrce-DDS agent Setup

You may follow the instructions to build the micro_ros_setup in [Building micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup#building)

Here is a summary of the instructions:

1. $ROS_DISTRO was set to foxy in our tests. 
   ```bash
   export ROS_DISTRO=foxy 
   ```
2. clone micro-ros setup repo and build it in a colcon workspace
   ```bash
   source /opt/ros/$ROS_DISTRO/setup.bash
   mkdir -p microros_ws/src
   cd microros_ws
   git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
   rosdep update && rosdep install --from-paths src --ignore-src -y
   colcon build
   source install/local_setup.bash
   ```
3. create and build the micro_ros_agent (more info in [Building micro-ROS-Agent](https://github.com/micro-ROS/micro_ros_setup#building-micro-ros-agent) )
   ```bash
   ros2 run micro_ros_setup create_agent_ws.sh
   ros2 run micro_ros_setup build_agent.sh
   source install/local_setup.sh
   ```

4. Run the agent (this is the step you will repeat for every SITL):

   ```bash
   cd microros_ws
   source install/local_setup.sh
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

   It should respond with:
   ```bash
   [1701917075.667887] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 8888
   [1701917075.668439] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
   ```

5. To make it easy to run the agent. We can use a bash script and add alias for it. Sample script is in `scripts/start_microros_px4.bash`. you will need to be in `microros_ws` before running this script.
   ``` bash
   cd microros_ws
   source ../scripts/start_microros_px4.bash
   ``` 
