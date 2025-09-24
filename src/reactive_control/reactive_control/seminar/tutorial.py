from sensor_msgs.msg import LaserScan
import math
import numpy as np
from .solution import (
    sol_apply_pd,
    sol_compute_future_distance_to_wall,
    sol_compute_throttle_command,
    sol_lidar_angle_to_distance,
)

THETA_DEG = 60
LOOKAHEAD = 0.6  # m
DESIRED_DISTANCE_FROM_WALL = 0.5  # m
KP = 1.2
KD = 0.0


"""
Étapes :
1. Jouer avec les paramètres et observer le comportement.
2. Coder la fonction lidar_angle_to_distance.
3. Coder la fonction apply pd
4. Coder la fonction compute future distance to wall
5. Coder la fonction compute throttle command

"""

"""
Parameters:
    scan (LaserScan): array of mesurements from the lidar.
    angle_min (float) : start angle of the scan in rad
    angle_increment (float) : angular distance between mesurements in rad

Returns:
    throttle (float), steering (float)
"""


def lidar_angle_to_distance(
    theta_rad: float,
    scan: list[float],
    angle_min: float,
    angle_increment: float,
):
    # TODO
    return sol_lidar_angle_to_distance(theta_rad, scan, angle_min, angle_increment)


def compute_throttle_command(steering: float):
    # TODO
    return sol_compute_throttle_command()


def compute_future_distance_to_wall(a: float, b: float, theta: float, lookahead: float):
    # TODO
    return sol_compute_future_distance_to_wall(a, b, theta, lookahead)


def apply_pd(error: float, kp: float, kd: float, last_error: float, delta_t: float | None):
    # TODO
    return sol_apply_pd(error, kp, kd, last_error, delta_t)
