import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from .seminar.tutorial import (
    lidar_angle_to_distance,
    compute_throttle_command,
    compute_future_distance_to_wall,
    apply_pd,
    THETA_DEG,
    LOOKAHEAD,
    DESIRED_DISTANCE_FROM_WALL,
    KP,
    KD,
)


def is_valid_lidar_scan(scan: float) -> bool:
    return not math.isinf(scan) and not math.isnan(scan)


class WallFollowNode(Node):
    def __init__(self):
        super().__init__("wall_follow_node")

        self.create_subscription(LaserScan, "autodrive/roboracer_1/lidar", self.lidar_callback, 10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, "drive", 10)
        self.lidar_viz_pub = self.create_publisher(Marker, "lidar_ray_viz", 10)

        self.declare_parameter("lidar_viz_angle", 0.0)

        self.last_time = None
        self.last_steering = 0.0
        self.last_throttle = 0.0
        self.last_error = 0.0

    def lidar_callback(self, scan: LaserScan):
        viz_angle = np.radians(self.get_parameter("lidar_viz_angle").get_parameter_value().double_value)
        self.publish_lidar_ray(scan, viz_angle)

        lidar_range_array: list[float] = scan.ranges  # type: ignore
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment

        theta = np.radians(THETA_DEG)
        theta_b = -np.pi / 2.0
        theta_a = theta_b + theta
        a = lidar_angle_to_distance(theta_a, lidar_range_array, angle_min, angle_increment)
        b = lidar_angle_to_distance(theta_b, lidar_range_array, angle_min, angle_increment)

        if not is_valid_lidar_scan(a) or not is_valid_lidar_scan(b):
            self.get_logger().warn("Invalid lidar scan, repeating last command")
            self.send_control_command(self.last_throttle, self.last_steering)
            return

        # Desired correction from wall follow equations
        D_tp1 = compute_future_distance_to_wall(a, b, theta, LOOKAHEAD)
        error = DESIRED_DISTANCE_FROM_WALL - D_tp1

        # PID control
        dt = None
        if self.last_time is not None:
            dt = self.get_clock().now().nanoseconds - self.last_time

        steering = apply_pd(error, KP, KD, self.last_error, dt)
        throttle = compute_throttle_command(steering)

        # Updating variables
        self.last_steering = steering
        self.last_throttle = throttle
        self.last_time = self.get_clock().now().nanoseconds
        self.last_error = error

        self.send_control_command(throttle, steering)

    def send_control_command(self, throttle: float, steering: float):
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.frame_id = "base_link"
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()

        ackermann_msg.drive.speed = throttle
        ackermann_msg.drive.steering_angle = steering

        self.drive_pub.publish(ackermann_msg)

    def publish_lidar_ray(self, scan: LaserScan, angle: float):

        distance = lidar_angle_to_distance(angle, scan.ranges, scan.angle_min, scan.angle_increment)  # type: ignore

        start = Point()
        start.x = 0.0
        start.y = 0.0
        start.z = 0.0

        end = Point()
        end.x = distance * math.cos(angle)
        end.y = distance * math.sin(angle)
        end.z = 0.0

        marker = Marker()
        marker.header.frame_id = "lidar"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.points = [start, end]

        self.lidar_viz_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
