#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <ackermann_msgs/msg/detail/ackermann_drive_stamped__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float32.hpp>

namespace autodrive_bridge {
class AckermannToAutodriveNode : public rclcpp::Node {
public:
  AckermannToAutodriveNode(const rclcpp::NodeOptions &options)
      : Node("ackermann_to_autodrive_node", options) {
    // Values taken from autodrive technical docs
    max_speed = this->declare_parameter("max_speed", 22.8);
    max_steering_angle = this->declare_parameter("max_steering_angle", 0.5236);

    drive_sub =
        this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive", 1,
            std::bind(&AckermannToAutodriveNode::drive_callback, this,
                      std::placeholders::_1));

    throttle_pub = this->create_publisher<std_msgs::msg::Float32>(
        "autodrive/roboracer_1/throttle_command", 1);
    steering_pub = this->create_publisher<std_msgs::msg::Float32>(
        "autodrive/roboracer_1/steering_command", 1);
  };

private:
  void drive_callback(
      const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    double speed = msg->drive.speed;
    double steering_angle = msg->drive.steering_angle;

    if (abs(speed) > max_speed) {
      RCLCPP_WARN(this->get_logger(),
                  "Speed command exceeds maximum speed. Clipping to %f",
                  max_speed);
      speed = copysign(max_speed, speed);
    }
    if (abs(steering_angle) > max_steering_angle) {
      RCLCPP_WARN(
          this->get_logger(),
          "Steering command exceeds maximum steering angle. Clipping to "
          "%f",
          max_steering_angle);
      steering_angle = copysign(max_steering_angle, steering_angle);
    }

    std_msgs::msg::Float32 throttle_msg;
    throttle_msg.data = msg->drive.speed / max_speed;

    std_msgs::msg::Float32 steering_msg;
    steering_msg.data = msg->drive.steering_angle / max_steering_angle;

    throttle_pub->publish(throttle_msg);
    steering_pub->publish(steering_msg);
  };

  double max_speed;
  double max_steering_angle;

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      drive_sub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub;
};
}; // namespace autodrive_bridge

#include "rclcpp_components/register_node_macro.hpp" // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(autodrive_bridge::AckermannToAutodriveNode)