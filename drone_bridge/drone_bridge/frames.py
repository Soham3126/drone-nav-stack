"""All ROS <-> ArduPilot frame conversions (guidebook 2.3). Tested by test_frames.py (B0).

ROS map frame: x forward, y left, z up (FLU), yaw anticlockwise positive.
ArduPilot local frame: north-east-down (NED). Indoors, "north" is simply the map x axis,
because EKF3 takes its heading from our pose.

  forward  x        -> north =  x
  sideways y (left) -> east  = -y
  height   z (up)   -> down  = -z
  yaw               -> yaw_ned = -yaw
  roll / pitch      -> roll / -pitch
"""
import math


def wrap(a):
    """Angle wrapped to (-pi, pi]. Uses atan2(sin, cos) so it is safe near +-180 deg."""
    w = math.atan2(math.sin(a), math.cos(a))
    return math.pi if w == -math.pi else w


def flu_to_ned_xyz(x, y, z):
    return x, -y, -z


def ned_to_flu_xyz(n, e, d):
    return n, -e, -d


def flu_to_ned_rpy(roll, pitch, yaw):
    return wrap(roll), wrap(-pitch), wrap(-yaw)


def ned_to_flu_rpy(roll, pitch, yaw):
    return wrap(roll), wrap(-pitch), wrap(-yaw)


def quat_to_rpy(qx, qy, qz, qw):
    """ZYX Euler angles (roll, pitch, yaw) from a unit quaternion."""
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return roll, pitch, math.atan2(siny, cosy)


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def pose_flu_to_ned(x, y, z, roll, pitch, yaw):
    """map -> base_link (FLU) as (n, e, d, roll, pitch, yaw_ned): the VISION_POSITION_ESTIMATE fields."""
    return flu_to_ned_xyz(x, y, z) + flu_to_ned_rpy(roll, pitch, yaw)


def pose_ned_to_flu(n, e, d, roll, pitch, yaw):
    return ned_to_flu_xyz(n, e, d) + ned_to_flu_rpy(roll, pitch, yaw)


def rotate2d(theta, x, y):
    """Rotate the vector (x, y) by theta (counter-clockwise in the plane it is expressed in)."""
    c, s = math.cos(theta), math.sin(theta)
    return c * x - s * y, s * x + c * y
