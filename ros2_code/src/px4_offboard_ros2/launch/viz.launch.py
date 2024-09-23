import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Set the path to the am_description package.
    robot_description_pkg = FindPackageShare(package='am_description').find('am_description')
    package_dir = get_package_share_directory('px4_offboard_ros2')

    # Set the path to the URDF file
    default_urdf_model_path = os.path.join(robot_description_pkg, 'urdf/hex_x.urdf')


    # Launch configuration variables specific to simulation
    urdf_model = LaunchConfiguration('urdf_model')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments  
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model',
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

    # Specify the actions

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': use_sim_time,
    'robot_description': Command(['xacro ', urdf_model])}],
    arguments=[default_urdf_model_path])


    # Launch visualization marker publisher
    start_viz_markers_cmd =  Node(
        package='px4_offboard_ros2',
        namespace='px4_offboard_ros2',
        executable='visualizer',
        name='visualizer')

    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(package_dir, 'drone.rviz')]])

    start_plotjuggler_cmd = Node(
        package='plotjuggler',
        namespace='px4_offboard_ros2',
        executable='plotjuggler',
        name='plots' )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)  
    ld.add_action(declare_use_sim_time_cmd)

    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_viz_markers_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_plotjuggler_cmd)

    return ld
