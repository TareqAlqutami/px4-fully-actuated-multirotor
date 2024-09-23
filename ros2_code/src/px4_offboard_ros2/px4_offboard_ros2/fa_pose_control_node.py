#!/usr/bin/env python

"""
PX4 Offboard Pose control in ROS2. The controller publishes attitude setpoints (attitude and 3D thrust) supporting fully-actuated vehicles.
The position controller controls the thrust while the attitude control is done in PX4 given the desired attitude.
Warning: Asking for too much 3D thrust will resulted in degraded attitude stabilization and may result in a crush. 
Tested on PX4 main (~v1.15) and ros2 Foxy (ubuntu 20.04)
Author: Tareq Alqutami (tareqaziz2010@gmail.com)
Dec 2023
"""

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rcl_interfaces.msg import SetParametersResult
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleAttitudeSetpoint, VehicleCommand
from px4_msgs.msg import VehicleStatus, VehicleAttitude, VehicleLocalPosition, HoverThrustEstimate

from px4_offboard_ros2.PIDn import PIDn
from px4_offboard_ros2.px4_transforms import px4_to_ros_local_frame, px4_to_ros_orientation, ros_to_px4_orientation, ros_to_px4_body_vector

class OffboardPoseControl(Node):
    def __init__(self):
        super().__init__('fa_pose_control_py')

        self.offboard_control_mode = ['attitude']
        self.engage_px4_offboard_mode = False # don't engage in offboard mode in startup, wait for user signal
        self.hover_thrust = np.array([0.0, 0.0, 0.5]) # in inertial frame

        # tracking variables
        self.att = np.array([1.0, 0.0, 0.0, 0.0]) # w,x,y,z
        self.att_sp = np.array([1.0, 0.0, 0.0, 0.0]) # w,x,y,z
        self.att_rpy = np.zeros(3) # r,p,y in degrees
        self.att_sp_rpy = np.zeros(3) # r,p,y in degrees
        self.position = np.zeros(3)
        self.position_sp = np.zeros(3)
        self.vel = np.zeros(3)
        self.acc = np.zeros(3)
        self.px4_vehicle_state = VehicleStatus()
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.hover_thrust_estimate = 0.5 # using PX4 estimator
        self.control_effort = np.zeros(3)
        self.p_effort = np.zeros(3)
        self.i_effort = np.zeros(3)
        self.d_effort = np.zeros(3)

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # px4 publishers
        self.px4_offboard_mode_pub = self.create_publisher(
            OffboardControlMode,'/fmu/in/offboard_control_mode', qos_profile)
        self.px4_att_sp_pub = self.create_publisher(
            VehicleAttitudeSetpoint,'/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.px4_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # px4 subscribers
        self.px4_status_sub = self.create_subscription(
            VehicleStatus,'/fmu/out/vehicle_status', self.px4_status_callback, qos_profile)
        self.px4_att_sub = self.create_subscription(
            VehicleAttitude,'/fmu/out/vehicle_attitude', self.px4_attitude_callback, qos_profile)
        self.px4_local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.px4_local_position_callback, qos_profile)
        self.px4_local_position_sub = self.create_subscription(
            HoverThrustEstimate, '/fmu/out/hover_thrust_estimate',
            self.px4_hover_thrust_callback, qos_profile)

        # setpoint subscribers
        self.target_pose_sub = self.create_subscription(
            PoseStamped,'/target_pose', self.target_pose_callback,10)


        # setpoints and state publishers
        self.att_sp_rpy_pub = self.create_publisher(
            Vector3Stamped,'/att_sp_rpy', qos_profile)
        self.att_rpy_pub = self.create_publisher(
            Vector3Stamped,'/att_rpy', qos_profile)
        self.att_rpy_ned_pub = self.create_publisher(
            Vector3Stamped,'/att_rpy_ned', qos_profile)
        self.vel_pub = self.create_publisher(
            Vector3Stamped,'/linear_vel', qos_profile)
        self.position_sp_pub = self.create_publisher(
            Vector3Stamped,'/position_sp', qos_profile)
        self.position_pub = self.create_publisher(
            Vector3Stamped,'/position', qos_profile)
        self.control_effort_pub = self.create_publisher(
            Vector3Stamped,'/control_effort', qos_profile)
        self.p_effort_pub = self.create_publisher(
            Vector3Stamped,'/p_effort', qos_profile)
        self.i_effort_pub = self.create_publisher(
            Vector3Stamped,'/i_effort', qos_profile)
        self.d_effort_pub = self.create_publisher(
            Vector3Stamped,'/d_effort', qos_profile)

        # controller params
        # aggressive tuning for t960_tilt30, tested with helix trajectory
        # self.kp = np.array([0.8, 0.8, 0.8])
        # self.ki = np.array([0.002,0.002,0.004])
        # self.kd = np.array([0.04,0.04,0.04])
        # tuning for tilted_hex
        self.kp = np.array([0.11,0.11,0.12])
        self.ki = np.array([0.001,0.001,0.002])
        self.kd = np.array([0.1,0.1,0.08])
        
        self.out_max = np.array([1.0, 1.0, 1.0]) # PX4 normalized acceleration limits
        self.out_min = np.array([-1.0, -1.0, -1.0]) # PX4 normalized acceleration limits
        self.derivative_on_state = True
        self.use_measured_dot = True

        # declare ros parameters and callback for controller gains and settings
        # dynamic reconfigure can be used to perform online tuning
        self.declare_ros_params()
        self.add_on_set_parameters_callback(self.ros_param_callback)

        # controllers
        self.position_control = PIDn(3, self.kp, self.ki, self.kd,
                                    self.out_max, self.out_min,True,
                                    self.derivative_on_state, self.use_measured_dot)

        control_period = 0.02  # 0.02 seconds = 50hz
        self.prev_time = self.get_clock().now().nanoseconds
        self.control_loop_timer = self.create_timer(control_period,self.control_loop)

    def px4_status_callback(self, msg: VehicleStatus) -> None:
        """ Callback for PX4 vehicle status"""
        if self.nav_state != msg.nav_state:      
            if msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().info("changed to offboard flight mode")
            else:
                self.get_logger().info(f"changed mode to {msg.nav_state}",throttle_duration_sec=3)

        self.nav_state = msg.nav_state
        self.px4_vehicle_state = msg

    def px4_attitude_callback(self,msg:VehicleAttitude) -> None:
        """ Callback for PX4 attitude. 
        PX4 attitude is in body FRD (left-handed) while ROS is is ENU reference frame.
        Transform to ROS ENU by converting the sign of y and z values"""
        q_enu = px4_to_ros_orientation(msg.q)
        self.att[0] = q_enu[0] # w
        self.att[1] = q_enu[1] # x
        self.att[2] = q_enu[2] # y
        self.att[3] = q_enu[3] # z
        ypr_rad_enu = Rotation.from_quat([q_enu[1], q_enu[2], q_enu[3], q_enu[0]]).as_euler('zyx') #scalar last
        rpy = np.array([ypr_rad_enu[2], ypr_rad_enu[1], ypr_rad_enu[0]])
        self.att_rpy = rpy

    def px4_local_position_callback(self, msg:VehicleLocalPosition) -> None:
        """ Callback for PX4 local_position. 
        PX4 attitude is in body FRD (left-handed) while ROS is is ENU reference frame.
        Transform to ROS ENU by converting the sign of y and z values"""
        self.position = px4_to_ros_local_frame(np.array([msg.x, msg.y, msg.z]))
        self.vel = px4_to_ros_local_frame(np.array([msg.vx, msg.vy, msg.vz]))
        self.acc = px4_to_ros_local_frame(np.array([msg.ax, msg.ay, msg.az]))

    def px4_hover_thrust_callback(self,msg:HoverThrustEstimate) -> None:
        """ Callback for PX4 hover thrust estimate. This is only published after takeoff 
        and requires adding the topic in PX4 firmware to dds_topics.yaml"""
        # TODO check variance before accepting new estimates
        self.hover_thrust_estimate = msg.hover_thrust

    def publish_px4_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.px4_command_pub.publish(msg)

    def arm(self) -> None:
        """Send an arm command to the vehicle."""
        self.publish_px4_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self) -> None:
        """Send a disarm command to the vehicle."""
        self.publish_px4_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self) -> None:
        """Switch to offboard mode."""
        # set set-point to current state to avoid jumps
        self.position_sp = self.position
        self.publish_px4_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self) -> None:
        """Switch to land mode."""
        self.publish_px4_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def target_pose_callback(self, msg:PoseStamped) -> None:
        """ callback to update the target pose used in position and attitude control.
        The data received is assumed to be in ENU frame"""
        self.position_sp[0] = msg.pose.position.x
        self.position_sp[1] = msg.pose.position.y
        self.position_sp[2] = msg.pose.position.z
        self.att_sp[0] = msg.pose.orientation.w
        self.att_sp[1] = msg.pose.orientation.x
        self.att_sp[2] = msg.pose.orientation.y
        self.att_sp[3] = msg.pose.orientation.z

        att_sp = Rotation.from_quat([self.att_sp[1], self.att_sp[2], self.att_sp[3], self.att_sp[0]]) #scalar last
        ypr = att_sp.as_euler('zyx')
        rpy = np.array([ypr[2], ypr[1], ypr[0]])
        self.att_sp_rpy = rpy

    def declare_ros_params(self):
        """ Declare ros parameters"""
        self.get_logger().info("Declaring parameters")
        self.declare_parameter("x_kp", self.kp[0])
        self.declare_parameter("y_kp", self.kp[1])
        self.declare_parameter("z_kp", self.kp[2])
        self.declare_parameter("x_ki", self.ki[0])
        self.declare_parameter("y_ki", self.ki[1])
        self.declare_parameter("z_ki", self.ki[2])
        self.declare_parameter("x_kd", self.kd[0])
        self.declare_parameter("y_kd", self.kd[1])
        self.declare_parameter("z_kd", self.kd[2])
        self.declare_parameter("derivative_on_measurement",self.derivative_on_state)
        self.declare_parameter("use_measured_derivative",self.use_measured_dot)
        self.declare_parameter("engage_px4_offboard_mode",False)

    def get_ros_params(self):
        """Update local variables from ros parameters"""
        self.kp = np.array([self.get_parameter("x_kp").value,
                                self.get_parameter("y_kp").value,
                                self.get_parameter("z_kp").value
                                ])
        self.ki = np.array([self.get_parameter("x_ki").value,
                                self.get_parameter("y_ki").value,
                                self.get_parameter("z_ki").value
                                ])
        self.kd = np.array([self.get_parameter("x_kd").value,
                                self.get_parameter("y_kd").value,
                                self.get_parameter("z_kd").value
                                ])
        self.derivative_on_state = self.get_parameter("derivative_on_measurement").value
        self.use_measured_dot = self.get_parameter("use_measured_derivative").value

    def update_control_params(self):
        """Update controllers parameters and reset them"""
        self.position_control.set_gains(self.kp,self.ki, self.kd)
        self.position_control.set_derivative_method(self.derivative_on_state, self.use_measured_dot)
        self.position_control.reset()

    def ros_param_callback(self,params:list):
        """Callback when ros parameters change"""
        for param in params:
            if param.name == 'x_kp':
                self.kp[0] = param.value
                self.get_logger().info(f"updated x_kp to {self.kp[0]}")
            elif param.name == 'x_ki':
                self.ki[0] = param.value
                self.get_logger().info(f"updated x_ki to {self.ki[0]}")
            elif param.name == 'x_kd':
                self.kd[0] = param.value
                self.get_logger().info(f"updated x_kd to {self.kd[0]}")
            elif param.name == 'y_kp':
                self.kp[1] = param.value
                self.get_logger().info(f"updated y_kp to {self.kp[1]}")
            elif param.name == 'y_ki':
                self.ki[1] = param.value
                self.get_logger().info(f"updated y_ki to {self.ki[1]}")
            elif param.name == 'y_kd':
                self.kd[1] = param.value
                self.get_logger().info(f"updated y_kd to {self.kd[1]}")
            elif param.name == 'z_kp':
                self.kp[2] = param.value
                self.get_logger().info(f"updated z_kp to {self.kp[2]}")
            elif param.name == 'z_ki':
                self.ki[2] = param.value
                self.get_logger().info(f"updated z_ki to {self.ki[2]}")
            elif param.name == 'z_kd':
                self.kd[2] = param.value
                self.get_logger().info(f"updated z_kd to {self.kd[2]}")
            elif param.name == 'derivative_on_measurement':
                self.derivative_on_state = param.value
                self.get_logger().info(f"updated derv_on_state to {self.derivative_on_state}")
            elif param.name == 'use_measured_derivative':
                self.use_measured_dot = param.value
                self.get_logger().info(f"updated use_measured_derivative to {self.use_measured_dot}")

            elif param.name == 'engage_px4_offboard_mode':
                self.engage_px4_offboard_mode = param.value
                self.get_logger().info(f"updated engage_px4_offboard_mode to {self.engage_px4_offboard_mode}")
                self.engage_offboard_mode()

        self.update_control_params()
        return SetParametersResult(successful=True)

    def publish_px4_offboard_control_mode(self, modes:list) -> None:
        """publish px4 offboard control mode. 

        Args:
            modes (list): list of modes to turn on
        """
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True if 'position' in modes else False
        offboard_msg.velocity = True if 'velocity' in modes else False
        offboard_msg.acceleration = True if 'acceleration' in modes else False
        offboard_msg.attitude = True if 'attitude' in modes else False
        offboard_msg.body_rate = True if 'body_rate' in modes else False
        self.offboard_control_mode = modes            
        self.px4_offboard_mode_pub.publish(offboard_msg)

    def publish_states(self) -> None:
        """Publish states for monitoring, plotting and troubleshooting.
        """
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        rpy_msg = Vector3Stamped()
        rpy_msg.vector.x = np.rad2deg(self.att_sp_rpy[0])
        rpy_msg.vector.y = np.rad2deg(self.att_sp_rpy[1])
        rpy_msg.vector.z = np.rad2deg(self.att_sp_rpy[2])
        self.att_sp_rpy_pub.publish(rpy_msg)

        rpy_msg = Vector3Stamped()
        rpy_msg.vector.x = np.rad2deg(self.att_rpy[0])
        rpy_msg.vector.y = np.rad2deg(self.att_rpy[1])
        rpy_msg.vector.z = np.rad2deg(self.att_rpy[2])
        self.att_rpy_pub.publish(rpy_msg)
        
        vel_msg = Vector3Stamped()
        vel_msg.vector.x = self.vel[0]
        vel_msg.vector.y = self.vel[1]
        vel_msg.vector.z = self.vel[2]
        self.vel_pub.publish(vel_msg)

        pos_msg = Vector3Stamped()
        pos_msg.vector.x = self.position[0]
        pos_msg.vector.y = self.position[1]
        pos_msg.vector.z = self.position[2]
        self.position_pub.publish(pos_msg)

        pos_msg = Vector3Stamped()
        pos_msg.vector.x = self.position_sp[0]
        pos_msg.vector.y = self.position_sp[1]
        pos_msg.vector.z = self.position_sp[2]
        self.position_sp_pub.publish(pos_msg)
        
        effort_msg = Vector3Stamped()
        effort_msg.vector.x = self.control_effort[0]
        effort_msg.vector.y = self.control_effort[1]
        effort_msg.vector.z = self.control_effort[2]
        self.control_effort_pub.publish(effort_msg)

        effort_msg = Vector3Stamped()
        effort_msg.vector.x = self.p_effort[0]
        effort_msg.vector.y = self.p_effort[1]
        effort_msg.vector.z = self.p_effort[2]
        self.p_effort_pub.publish(effort_msg)

        effort_msg = Vector3Stamped()
        effort_msg.vector.x = self.i_effort[0]
        effort_msg.vector.y = self.i_effort[1]
        effort_msg.vector.z = self.i_effort[2]
        self.i_effort_pub.publish(effort_msg)

        effort_msg = Vector3Stamped()
        effort_msg.vector.x = self.d_effort[0]
        effort_msg.vector.y = self.d_effort[1]
        effort_msg.vector.z = self.d_effort[2]
        self.d_effort_pub.publish(effort_msg)
     
    def control_loop(self) -> None:
        """Callback function for the timer that runs the control loop at desired rate"""
        # Publish offboard control modes
        self.publish_px4_offboard_control_mode(self.offboard_control_mode)
        # publish states
        self.publish_states()

        # in attitude offboard mode, use our custom controller
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            time_now = self.get_clock().now().nanoseconds
            dt = (time_now - self.prev_time)/1000000000.0
            self.prev_time = time_now
            # PID position(x,y,z) control

            acc_out = self.position_control.update(dt, self.position, self.position_sp, self.vel)
            acc_out = np.clip(acc_out,self.out_min,self.out_max)
            self.control_effort = acc_out
            p_err, i_err, d_err = self.position_control.get_current_errors()
            self.p_effort, self.i_effort, self.d_effort = self.position_control.get_current_efforts()
            # self.get_logger().info(f"p_err={p_err.round(2)}, acc={acc_out.round(2)}",
            #                         throttle_duration_sec=2)

            self.get_logger().info(f"pos={self.position.round(2)}, vel={self.vel.round(2)}, att={np.rad2deg(self.att_rpy).round(2)}",
                                    throttle_duration_sec=0.5)
            # Compute and saturate thrust values
            self.hover_thrust[2] = self.hover_thrust_estimate
            thrust_inertial =  self.hover_thrust + acc_out
            thrust_inertial = thrust_inertial.clip(-1.0,1.0)

            # ========== fully-actuated ======
            att_sp = self.att_sp
            # rotate thrust from inertial to body frame
            curr_att = Rotation.from_quat(
                [self.att[1], self.att[2], self.att[3], self.att[0]]) #scalar last
            R = curr_att.as_matrix()
            thrust_body = np.matmul(R.transpose(), thrust_inertial)
            th_sp = thrust_body

            att_sp_ned = ros_to_px4_orientation(att_sp)
            thrust_body_frd = ros_to_px4_body_vector(th_sp)
            # send AttitudeSetpoint to PX4
            att_sp_msg = VehicleAttitudeSetpoint()
            att_sp_msg.q_d[0] = att_sp_ned[0]
            att_sp_msg.q_d[1] = att_sp_ned[1]
            att_sp_msg.q_d[2] = att_sp_ned[2]
            att_sp_msg.q_d[3] = att_sp_ned[3]
            att_sp_msg.thrust_body[0] = thrust_body_frd [0]
            att_sp_msg.thrust_body[1] = thrust_body_frd [1]
            att_sp_msg.thrust_body[2] = thrust_body_frd [2]
            self.px4_att_sp_pub.publish(att_sp_msg)

def main(args=None):
    """Main function to execute"""
    rclpy.init(args=args)

    offboard_control = OffboardPoseControl()
    
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
