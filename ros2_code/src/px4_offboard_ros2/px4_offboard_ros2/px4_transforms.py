
from scipy.spatial.transform import Rotation
import numpy as np


# this file replicates some of the functionalities in https://github.com/PX4/px4_ros_com/blob/main/include/px4_ros_com/frame_transforms.h


# Important notes
# - Scipy is used to perform the transformations
# - Scipy represents quaternion as (x,y,z,w) while px4 and Eigen use (w,x,y,z). The constants are in scipy format but the transformation functions are expecting px4 format and return px4 format (w,x,y,z)
# - ENU to NED transform is the same as NED to ENU
# - The orientation transform ENU<>NED first transforms ENU<>NED then transform the body baselink  FRD<>FLU
# - the transform of vectors (i.e. position, velocity) between ENU<>NED is done by swapping x,y values and inverting z value.


AIRCRAFT_BASELINK = Rotation.from_euler('x',np.pi)
NED_ENU =  Rotation.from_euler('xyz',[np.pi, 0, np.pi/2])

AIRCRAFT_BASELINK_Q = AIRCRAFT_BASELINK.as_quat() # (x,y,z,w)
AIRCRAFT_BASELINK_R = AIRCRAFT_BASELINK.as_matrix()

NED_ENU_Q = NED_ENU.as_quat() # the same as ENU_NED
NED_ENU_R = NED_ENU.as_matrix()

def orientation_ned2enu(q_ned:np.array)->np.array:
    """Convert orientation between NED and ENU reference frames.
    Orientation is represented as quaternion (w,x,y,z)

    Args:
        q_ned (np.array): quaternion in NED

    Returns:
        np.array: quaternion in ENU
    """
    q= Rotation.from_quat([q_ned[1], q_ned[2], q_ned[3], q_ned[0]]) # scipy format is (x,y,z,w)
    q_out = (NED_ENU*q).as_quat()
    return np.array([q_out[3], q_out[0], q_out[1], q_out[2]]) # w,x,y,z


def orientation_enu2ned(q_enu:np.array)->np.array:
    """Convert orientation between NED and ENU reference frames.
    Orientation is represented as quaternion (w,x,y,z)

    Args:
        q_enu (np.array): quaternion in ENU

    Returns:
        np.array: quaternion in NED
    """
    q= Rotation.from_quat([q_enu[1], q_enu[2], q_enu[3], q_enu[0]]) # scipy format is (x,y,z,w)
    q_out = (NED_ENU*q).as_quat()  
    return np.array([q_out[3], q_out[0], q_out[1], q_out[2]]) # w,x,y,z


def orientation_baselink2aircraft(q_enu:np.array)->np.array:
    """Convert orientation between baselink frame and aircraft frames.
    Orientation is represented as quaternion (w,x,y,z)

    Args:
        q_enu (np.array): quaternion in baselink frame

    Returns:
        np.array: quaternion in aircraft frame
    """
    q_baselink= Rotation.from_quat([q_enu[1], q_enu[2], q_enu[3], q_enu[0]]) # scipy format is (x,y,z,w)
    q_out = (q_baselink*AIRCRAFT_BASELINK).as_quat()
    return np.array([q_out[3], q_out[0], q_out[1], q_out[2]]) # w,x,y,z

def orientation_aircraft2baselink(q:np.array)->np.array:
    """Convert orientation between baselink frame and aircraft frames.
    Orientation is represented as quaternion (w,x,y,z)

    Args:
        q (np.array): quaternion in aircraft frame

    Returns:
        np.array: quaternion in baselink frame
    """
    q_aircraft= Rotation.from_quat([q[1], q[2], q[3], q[0]]) # scipy format is (x,y,z,w)
    q_out = (q_aircraft*AIRCRAFT_BASELINK).as_quat()
    return np.array([q_out[3], q_out[0], q_out[1], q_out[2]]) # w,x,y,z

def px4_to_ros_orientation(q:np.array)->np.array:
    """Transform from orientation represented in PX4 format to ROS format
    PX4 format is aircraft to NED
    ROS format is baselink to ENU

    Two steps conversion:
        1. aircraft to NED is converted to aircraft to ENU (NED_to_ENU conversion)
        2. aircraft to ENU is converted to baselink to ENU (aircraft_to_baselink conversion)

    Args:
        q (np.array): orientation quaternion in PX4 format

    Returns:
        np.array: orientation quaternion in ROS format
    """
    return orientation_aircraft2baselink(orientation_ned2enu(q))

def ros_to_px4_orientation(q:np.array)->np.array:
    """Transform from orientation represented in ROS format to PX4 format
    PX4 format is aircraft to NED
    ROS format is baselink to ENU

    Two steps conversion:
        1. baselink to ENU is converted to baselink to NED (ENU_to_NED conversion)
        2. baselink to ENU is converted to aircraft to ENU (baselink_to_aircraft conversion)

    Args:
        q (np.array): orientation quaternion in ROS format

    Returns:
        np.array: orientation quaternion in PX4 format
    """
    return orientation_baselink2aircraft(orientation_enu2ned(q))


def px4_to_ros_body_vector(v:np.array)->np.array:
    v_out = np.dot(v,AIRCRAFT_BASELINK_R)
    # or shortcut negative y and z
    # v_out = np.array([v[0], -v[1], -v[2]])
    return v_out

def ros_to_px4_body_vector(v:np.array)->np.array:
    v_out = np.dot(v,AIRCRAFT_BASELINK_R)
    # or shortcut negative y and z
    # v_out = np.array([v[0], -v[1], -v[2]])
    return v_out

def vector_ned2enu(v:np.array):
    """Transform vector (x,y,z) expressed in NED frame to be expressed in ENU frame
    The transformation is done by swapping x and y values and inverting z value

    Args:
        v (np.array): vector in NED frame

    Returns:
        np.array: vector in ENU frame
    """
    return np.array([v[1], v[0], -v[2]])

def vector_enu2ned(v:np.array):
    """Transform vector (x,y,z) expressed in ENU frame to be expressed in NED frame
    The transformation is done by swapping x and y values and inverting z value

    Args:
        v (np.array): vector in ENU frame

    Returns:
        np.array: vector in NED frame
    """
    return np.array([v[1], v[0], -v[2]])


def px4_to_ros_local_frame(v):
    """Transform a vector expressed in PX4 local frame to ROS local frame

    Args:
        v (np.array): x,y,z values of vector in PX$

    Returns:
        np.array: x,y,z values of vector in ROS
    """
    return vector_ned2enu(v)

def ros_to_px4_local_frame(v):
    """Transform a vector expressed in ROS local frame to PX4 local frame

    Args:
        v (np.array): x,y,z values of vector in ROS

    Returns:
        np.array: x,y,z values of vector in PX4
    """
    return vector_enu2ned(v)
