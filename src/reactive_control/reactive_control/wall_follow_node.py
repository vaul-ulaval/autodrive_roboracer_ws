import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# Lab constants
THETA_DEG = 60
LOOKAHEAD = 0.6  # m
DESIRED_DISTANCE_FROM_WALL = 0.5  # m
INTEGRAL_WINDOW_SIZE = 10
KP = 1.2
KD = 0.0
KI = 0.0


def angle_to_distance(theta_rad: float, lidar_array: list[float], angle_min: float, angle_increment: float):
    index = int((theta_rad - angle_min) / angle_increment)
    distance = lidar_array[index]

    return distance


def is_valid_lidar_scan(scan: float) -> bool:
    return not math.isinf(scan) and not math.isnan(scan)


class WallFollowNode(Node):
    def __init__(self):
        super().__init__("wall_follow_node")

        self.create_subscription(LaserScan, "autodrive/roboracer_1/lidar", self.lidar_callback, 10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, "drive", 10)

        self.last_time = None
        self.last_steering = 0.0
        self.last_errors_window = np.array([])

    def lidar_callback(self, scan: LaserScan):
        steering = 0.0  # rad
        throttle = 2.2  # m/s

        lidar_range_array: list[float] = scan.ranges  # type: ignore
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment

        theta = np.radians(THETA_DEG)
        theta_b = -np.pi / 2.0
        theta_a = theta_b + theta
        a = angle_to_distance(theta_a, lidar_range_array, angle_min, angle_increment)
        b = angle_to_distance(theta_b, lidar_range_array, angle_min, angle_increment)

        if not is_valid_lidar_scan(a) or not is_valid_lidar_scan(b):
            self.get_logger().warn("Invalid lidar scan, repeating last command")
            self.send_control_command(throttle, self.last_steering)
            return

        alpha = math.atan((a * math.cos(theta) - b) / (a * math.sin(theta)))
        D_t = b * math.cos(alpha)
        D_tp1 = D_t + LOOKAHEAD * math.sin(alpha)

        error = DESIRED_DISTANCE_FROM_WALL - D_tp1

        # PID control
        if self.last_time is None:
            error_diff = 0.0
            error_integral = 0.0
        else:
            dt = self.get_clock().now().nanoseconds - self.last_time
            de = error - self.last_errors_window[-1]
            error_diff = de / dt
            error_integral = np.sum(self.last_errors_window) * dt

        steering = error * KP + error_integral * KI + error_diff * KD

        # Updating variables
        self.last_steering = steering
        self.last_time = self.get_clock().now().nanoseconds
        if len(self.last_errors_window) >= INTEGRAL_WINDOW_SIZE:
            self.last_errors_window[:-1] = self.last_errors_window[1:]
            self.last_errors_window[-1] = error
        else:
            self.last_errors_window = np.append(self.last_errors_window, error)

        self.send_control_command(throttle, steering)

    def send_control_command(self, throttle: float, steering: float):
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.frame_id = "base_link"
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()

        ackermann_msg.drive.speed = throttle
        ackermann_msg.drive.steering_angle = steering

        self.drive_pub.publish(ackermann_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
