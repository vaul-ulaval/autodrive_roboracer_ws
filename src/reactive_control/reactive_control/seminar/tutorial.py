from sensor_msgs.msg import LaserScan
import math
import numpy as np
from .solution import (
    sol_apply_pd,
    sol_compute_future_distance_to_wall,
    sol_compute_throttle_command,
    sol_lidar_angle_to_distance,
)


# TODO #1 Tweak these parameters!
THETA_DEG = 60
LOOKAHEAD = 0.6  # m
DESIRED_DISTANCE_FROM_WALL = 0.5  # m
KP = 1.2
KD = 0.0


def lidar_angle_to_distance(
    theta_rad: float,
    scan: list[float],
    angle_min: float,
    angle_increment: float,
):
    """
    Params:
        theta_rad (float): angle (in rad)
        scan (list of float): array of lidar measurements (in m)
        angle_min (float) : start angle of the scan (in rad)
        angle_increment (float) : angular distance between mesurements (in rad)
    Returns:
        distance (float): distance mesured by lidar at angle theta_rad (in m)
    """

    # TODO #2 comment next line and implement the function.
    return sol_lidar_angle_to_distance(theta_rad, scan, angle_min, angle_increment)


def apply_pd(error: float, kp: float, kd: float, last_error: float, delta_t: float | None):
    """
    Params:
        error (float): current distance to wall error (in m)
        kp (float): proportional gain
        kd (float): derivative gain
        last_error (float): last distance to wall error (in m)
        delta_t (float | None): time elapsed since last call (in s)
    Returns:
        steering_angle (float): steering angle in rad to correct the error
    """

    # TODO 3 comment next line and implement the function.
    return sol_apply_pd(error, kp, kd, last_error, delta_t)


def compute_future_distance_to_wall(a: float, b: float, theta: float, lookahead: float):
    """
    Params:
        a (float): distance to wall at angle THETA_DEG
        b (float): distance to wall at angle 90 degrees
        theta (float): angle (in rad)
        lookahead (float): lookahead distance (in m)
    Returns:
        future_distance (float): estimated future distance to wall considering the lookahead
    """

    # TODO 4 comment next line and implement the function.
    return sol_compute_future_distance_to_wall(a, b, theta, lookahead)


def compute_throttle_command(steering: float):
    # TODO 5 You can implement this function however you want! The goal is to maximise speed.
    return sol_compute_throttle_command()