#!/usr/bin/env python

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, Vector3Stamped, PoseStamped
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

class controlSigGenerator(Node):
    """
    generate control signal waveform
    """
    def __init__(self):
        super().__init__('control_sig_generator')
        

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # setpoints and state publishers
        self.att_sp_rpy_pub = self.create_publisher(
            Vector3Stamped,'/target_att_rpy', 10)
        self.vel_sp_pub = self.create_publisher(
            Twist,'/target_twist', 10)
        self.pose_sp_pub = self.create_publisher(
            PoseStamped,'/target_pose', 10)   

        self.force_sp_pub = self.create_publisher(
            Float32,'/target_fx', 10)

        # square signal waveform (velocity vector and euler angles attitude)
        # t: time in seconds since the start of this node"
        # value(x,y,z): value at time t with the same index"
        # self.mode = 'vel'
        # self.t     = np.array([  0,    2,   4,    6,    8,  10,  12,  14,   16,   18,   20,   22,   24,  26,   28])
        # self.x     = np.array([0.0,  0.0,  0.1, 0.2,  0.4,  0.2, 0.0, 0.0,-0.4, -0.4, -0.4, -0.2, -0.2,  0.0,  0.0])
        # self.y     = np.array([0.0,  0.0,  0.0, 0.1,  0.2,  0.4, 0.4, 0.0, 0.0, -0.4, -0.4, -0.2, -0.2,  0.0,  0.0])
        # self.z     = np.array([0.0,  0.0,  0.2, 0.4,  0.4,  0.2, 0.2,-0.2,-0.2,  0.0, -0.1, -0.3,  0.0,  0.0,  0.0])
        # self.roll  = np.array([0.0,  0.0,  0.0, 0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])
        # self.pitch = np.array([0.0,  0.0,  0.0, 0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])
        # self.yaw   = np.array([0.0,  0.0,  0.0, 0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])

        # self.mode = 'vel' # aggressive
        # self.t     = np.array([  0,    2,   4,    6,    8,  10,  12,  14,   16,   18,   20,   22,   24,  26,   28])
        # self.x     = np.array([0.0,  0.0,  0.1, 0.2,  0.4,  0.2, 0.0, 0.0,-0.4, -0.4, -0.4, -0.2, -0.2,  0.0,  0.0])*2.0
        # self.y     = np.array([0.0,  0.0,  0.0, 0.1,  0.2,  0.4, 0.4, 0.0, 0.0, -0.4, -0.4, -0.2, -0.2,  0.0,  0.0])*2.0
        # self.z     = np.array([0.0,  0.0,  0.2, 0.4,  0.4,  0.2, 0.2,-0.2,-0.2,  0.0, -0.1, -0.3,  0.0,  0.0,  0.0])*1.5
        # self.roll  = np.array([0.0,  0.0,  0.0, 0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])
        # self.pitch = np.array([0.0,  0.0,  0.0, 0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])
        # self.yaw   = np.array([0.0,  0.0,  0.0, 0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])

        # self.mode = 'pos' # attitude only for fully-actuated vehicle
        # self.t     = np.array([  0,    2,   4,    6,     8,    10,  12,  14,   16,   18,   20,   22,   24,  26])
        # self.x     = np.array([0.0,  0.0, 0.0,  0.0,   0.0,   0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])
        # self.y     = np.array([0.0,  0.0, 0.0,  0.0,   0.0,   0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])
        # self.z     = np.array([2.0,  2.0, 2.0,  2.0,   2.0,   2.0, 2.0, 2.0,  2.0,  2.0,  2.0,  2.0,  2.0,  2.0])
        # self.roll  = np.array([0.0,  4.0, 0.0, -5.0, -10.0, -15.0,-8.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])
        # self.pitch = np.array([0.0,  0.0, 0.0,  0.0,   0.0,   0.0, 0.0, 5.0,  0.0, -5.0,  0.0,  0.0,  0.0,  0.0])
        # self.yaw   = np.array([0.0,  0.0, 0.0,  0.0,   0.0,   0.0, 0.0, 0.0,  0.0,  0.0, 10.0,  0.0,-10.0,  0.0])      

        # self.mode = 'pos' # position only
        # self.t     = np.array([  0,    2,   4,    6,     8,    10,  12,  14,   16,   18,   20,   22,   24,  26])
        # self.x     = np.array([0.0,  0.3, 0.3,  0.3,   0.0,  -0.2, 0.2, 0.2,  0.2,  0.2,  0.2,  0.0,  0.0,  0.0])
        # self.y     = np.array([0.0,  0.0, 0.0,  0.3,   0.3,  -0.2, 0.2, 0.2,  0.2,  0.2,  0.2,  0.0,  0.0,  0.0])
        # self.z     = np.array([2.0,  2.0, 2.0,  2.0,   2.0,   2.0, 2.0, 2.5,  2.5,  2.5,  2.5,  2.0,  2.0,  2.0])
        # self.roll  = np.array([0.0,  0.0, 0.0,  0.0,   0.0,   0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])
        # self.pitch = np.array([0.0,  0.0, 0.0,  0.0,   0.0,   0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])
        # self.yaw   = np.array([0.0,  0.0, 0.0,  0.0,   0.0,   0.0, 0.0, 0.0, 90.0, 90.0,  0.0,  0.0,  0.0,  0.0])  


        self.mode = 'helix'
        self.angle = 0.0
        self.radius = 1.0
        self.delta_z = 0.0005 # control pitch of the helix
        self.delta_angle = 0.02 # control the angular speed of the circular path
        self.x = 0.0
        self.y = 0.0
        self.z = 1.5
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
 
        # self.mode = 'vel-helix'
        # self.angle = 0.0
        # self.radius = 1.0
        # self.delta_angle = 0.02 # control the angular speed of the circular path
        # self.x = 0.0
        # self.y = 0.0
        # self.z = 0.05
        # self.roll = 0.0
        # self.pitch = 0.0
        # self.yaw = 0.0
        
        # self.mode = 'att-sincos'
        # self.angle = 0.0
        # self.max_roll = 10.0 # degrees
        # self.max_pitch = 10.0 # degrees
        # self.max_yaw = 20.0 # degrees
        # self.omega = 0.5 # control the period
        # self.x = 0.0
        # self.y = 0.0
        # self.z = 1.5
        # self.roll = 0.0
        # self.pitch = 0.0
        # self.yaw = 0.0
        
      
        
        
        self.i = 0 # starting index
        self.vel = np.zeros(3) # initial values
        self.att = np.zeros(3) # initial values
        self.position = np.zeros(3) # initial values
        self.f_cmd = 0.0 # initial values
            
        control_period = 0.02  # 0.02 seconds = 50hz
        self.start_time = self.get_clock().now().nanoseconds
        self.prev_time = 0.0
        self.control_loop_timer = self.create_timer(control_period,self.control_loop)



    def pub_rpy_cmd(self,sig:np.array) -> None:
        # timestamp = int(self.get_clock().now().nanoseconds / 1000)
        rpy_msg = Vector3Stamped()
        rpy_msg.vector.x = sig[0]
        rpy_msg.vector.y = sig[1]
        rpy_msg.vector.z = sig[2]
        self.att_sp_rpy_pub.publish(rpy_msg)

    def pub_twist_cmd(self, sig:np.array) ->None:
        vel_msg = Twist()
        vel_msg.linear.x = sig[0]
        vel_msg.linear.y = sig[1]
        vel_msg.linear.z = sig[2]
        self.vel_sp_pub.publish(vel_msg)

    def pub_pose_cmd(self,position:np.array,rpy:np.array) -> None:
        q_tmp = Rotation.from_euler(
            'XYZ',[rpy[0], rpy[1], rpy[2]],degrees=True).as_quat()
        q = np.zeros(4)
        q[0] = q_tmp[3]
        q[1] = q_tmp[0]
        q[2] = q_tmp[1]
        q[3] = q_tmp[2]
        msg = PoseStamped()
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.w = q[0]
        msg.pose.orientation.x = q[1]
        msg.pose.orientation.y = q[2]
        msg.pose.orientation.z = q[3]
        self.pose_sp_pub.publish(msg)
        
    def pub_force_cmd(self, force:float) ->None:
        msg = Float32()
        msg.data = force
        self.force_sp_pub.publish(msg)    

    def control_loop(self) -> None:
        """Callback function for the timer that runs the control loop at desired rate"""
                
        t = (self.get_clock().now().nanoseconds - self.start_time) /1000_000_000.0
        dt = (t - self.prev_time)
        self.prev_time = t
        
        if self.mode=='vel':
            # time to update signals
            if self.i<len(self.t) and t>=self.t[self.i]:
                self.vel = np.array([self.x[self.i], self.y[self.i], self.z[self.i]])
                self.att = np.array([self.roll[self.i], self.pitch[self.i], self.yaw[self.i]])
                self.i+=1
            self.get_logger().info(f"publishing commands: vel={self.vel.round(3)}, att={self.att.round(3)}", throttle_duration_sec=0.5)        
            self.pub_rpy_cmd(self.att)
            self.pub_twist_cmd(self.vel)
        
        elif self.mode=='pos':
             # time to update signals
            if self.i<len(self.t) and t>=self.t[self.i]:
                self.position = np.array([self.x[self.i], self.y[self.i], self.z[self.i]])
                self.att = np.array([self.roll[self.i], self.pitch[self.i], self.yaw[self.i]])
                self.i+=1
            self.get_logger().info(f"publishing commands: att={self.att.round(3)}, position={self.position.round(3)}", throttle_duration_sec=0.5)        
            self.pub_pose_cmd(self.position, self.att)   
            

        elif self.mode=='vel-helix':
             # time to update signals
            self.x = self.radius * np.cos(self.angle) 
            self.y = self.radius * np.sin(self.angle) 
            self.angle += self.delta_angle
            
            vel = np.array([self.x, self.y, self.z])
            att = np.array([self.roll, self.pitch, self.yaw])
            self.get_logger().info(f"publishing commands: vel={vel.round(3)}, att={att.round(3)}", throttle_duration_sec=0.5)        
            self.pub_rpy_cmd(att)
            self.pub_twist_cmd(vel)
            
        elif self.mode=='helix':
             # time to update signals
            self.x = self.radius * np.cos(self.angle) 
            self.y = self.radius * np.sin(self.angle) 
            self.z += self.delta_z
            self.angle += self.delta_angle
            
            position = np.array([self.x, self.y, self.z])
            att = np.array([self.roll, self.pitch, self.yaw])
            self.get_logger().info(f"publishing commands: att={att.round(3)}, position={position.round(3)}", throttle_duration_sec=0.5)        
            self.pub_pose_cmd(position, att)   

        elif self.mode=='att-sincos':
             # time to update signals
            self.roll  = np.rad2deg(np.deg2rad(self.max_roll) * np.sin(self.omega*t))
            self.pitch = np.rad2deg(np.deg2rad(self.max_pitch) * np.sin(self.omega*t))
            self.yaw   = np.rad2deg(np.deg2rad(self.max_yaw) * np.sin(self.omega*t))
            
            position = np.array([self.x, self.y, self.z])
            att = np.array([self.roll, self.pitch, self.yaw])
            self.get_logger().info(f"publishing commands: att={att.round(3)}, position={position.round(3)}", throttle_duration_sec=0.5)        
            self.pub_pose_cmd(position, att)   
            
def main(args=None):
    """Main function to execute"""
    rclpy.init(args=args)

    sig_generator = controlSigGenerator()

    rclpy.spin(sig_generator)
    sig_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
