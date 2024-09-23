import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped


def thrust_to_attitude(acc_sp, yaw_sp):
    # Converts desired acceleration vector in inertial frame & yaw setpoint to desired attitude and thrust(in body frame)
    
    # # get the z axis of rotation matrix for gravity direction
    # z_w = np.array([0,0,1.0])
    # z_b = R@z_w # multiple this with any desired force vector in inertial frame to get force in body frame (underactuated z-axis)    

    # normalize desired force to get z axis
    F = acc_sp
    z=F/np.linalg.norm(F)
    # rotate by desired yaw
    x_C= np.array([np.cos(yaw_sp),np.sin(yaw_sp),0])
    # get y and x axes and form rotation matrix
    v = np.cross(z,x_C)
    y = v/np.linalg.norm(v)
    v = np.cross(y,z)
    x = v/np.linalg.norm(v)
    R= np.array([[x[0], y[0], z[0]],
                 [x[1], y[1], z[1]],
                 [x[2], y[2], z[2]]])

    quat = Rotation.from_matrix(R).as_quat()
    att_sp = np.array([quat[3], quat[0], quat[1], quat[2]])
    
    thrust = np.dot(acc_sp, z) # or dot product with z vector of current attitude? R@[0,0,1]
    return att_sp, thrust


def drone_arm_mass_inertia(L:float, n:int, arm_mass, prop_mass):
    #Inertia of an arm modeled as a tube of length L starting at the origin

    mt = arm_mass # mass of the tube
    mp = prop_mass # mass of one propeller group
    ma = mt + mp # total mass of an arm
    r1 = 0.0075  # inner radius of the tube
    r2 = 0.008 # outer radius of the tube
    #                                      ----^----
    #                                          |                          z
    #          Arm representation:        h{  |_|________________         |
    #                                         |__________________    x____|
    #                                          <--L/2--><--L/2-->
    
    ## inertia of a tube (arm)
    Itube = np.array([[mt*(r1**2+r2**2)/2,                          0 ,                           0],
                      [0                 , mt*(3*(r1**2+r2**2) + L**2)/12,                        0],
                      [0                 ,                          0, mt*(3*(r1**2+r2**2) + L**2)/12 ]])
    r = np.array([[-L/2, 0, 0]]).T # distance (from the center of the tube) at which we want the Inertia tensor
    Itube = Itube + mt*(np.linalg.norm(r)**2 *np.eye(3)-r*r.T) # parallel axis theorem (Steiner's rule) to get inertia at (0, 0, 0)

    ## Inertia of a propeller block modeled as a cylinder
    r3 = 0.03 # diameter of the cyl        -> r3 <-   
    h = 0.065 # height of the cyl         ----^----
    #                                    |-    |                         z
    #                                   h{    |_|_______________         |
    #    Propeller block:                |-    _________________    x____|
    #                                          <------ L-------> 
    # Average the rotational inertia about y- and z-
    # Inertia of a clylinder (propeller block)
    Ip_xx = mp*(3*(r3**2) + h**2)/12 # Ip_xx = Ip_yy
    Ip_zz = mp*(r3**2)/2
    Ip = np.array([[Ip_xx,                   0 ,                  0],
                  [      0, 0.5*(Ip_xx + Ip_zz),                  0],
                  [      0,                  0, 0.5*(Ip_xx + Ip_zz)]])
    r = np.array([[-L, 0, 0]]).T # distance (from the center of the propeller group) at which we want the Inertia tensor
    Ip = Ip + mp*(np.linalg.norm(r)**2*np.eye(3)-r*r.T)  # parallel axis theorem (Steiner's rule) to get inertia at (0, 0, 0)

    ## calculate the total inertia for all the arms
    interval = 2 * np.pi / n # interval between arms in normal n-copter configuration
    Iarms = np.zeros((3,3))

    for i in range(n):
        Rb=Rotation.from_rotvec(i*interval*np.array([0,0,1])).as_matrix() # rotation around Z-axi to get transform from arm to base
        Iarms = Iarms + np.matmul(np.matmul(Rb , (Itube+Ip)) ,Rb.T) # Add the inertias of the propeller and the tube (in the body frame)
    
    ## Total inertia and mass of the arms
    I = Iarms
    m = n * ma
    
    return m,I
      
      
def vec2PoseStamped(frame_id, position, attitude):
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = frame_id
    pose_msg.pose.orientation.w = attitude[0]
    pose_msg.pose.orientation.x = attitude[1]
    pose_msg.pose.orientation.y = attitude[2]
    pose_msg.pose.orientation.z = attitude[3]
    pose_msg.pose.position.x = position[0]
    pose_msg.pose.position.y = position[1]
    pose_msg.pose.position.z = position[2]
    return pose_msg



voltage = 0
pitch_correction = 0
roll_correction = 0

def thrust_newton_to_px4( f):
  # return -0.00035 * (f * f) + 0.03606908 * f + 0.08797804;
  # return -0.00025633 * (f * f) + 0.03147742 * f + 0.14074639;
  return 0.02693*f + 0.15
  # return 0.05*f + 0.1;


def thrust_newton_to_px4(f, r,  p):
  # return 0.02092705*f + 0.01660002*fabs(r) + 0.03636566*fabs(p) + 0.1860;
  return thrust_newton_to_px4(f) - np.abs(p) * pitch_correction - np.abs(r) * roll_correction

def thrust_newton_to_px4_real(f):
  pwm = (f - 286.8 + 15.48*voltage)/(-0.2563)
  return (pwm - 1000.0)/1000.0
