import math


def sol_lidar_angle_to_distance(
    theta_rad: float,
    scan: list[float],
    angle_min: float,
    angle_increment: float,
):
    index = int((theta_rad - angle_min) / angle_increment)
    distance = scan[index]

    return distance


def sol_compute_throttle_command():
    return 2.2


def sol_compute_future_distance_to_wall(
    a: float, b: float, theta: float, lookahead: float
):
    alpha = math.atan((a * math.cos(theta) - b) / (a * math.sin(theta)))
    D_t = b * math.cos(alpha)
    D_tp1 = D_t + lookahead * math.sin(alpha)
    return D_tp1


def sol_apply_pd(
    error: float, kp: float, kd: float, last_error: float, delta_t: float | None
):

    error_diff = 0.0
    if delta_t is not None:
        delta_e = error - last_error
        error_diff = delta_e / delta_t

    steering = error * kp + error_diff * kd
    return steering
