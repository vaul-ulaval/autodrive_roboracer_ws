#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

using namespace std::chrono_literals;
namespace mf = message_filters;

namespace autodrive_bridge {
class WheelOdometryNode : public rclcpp::Node {
public:
  WheelOdometryNode(const rclcpp::NodeOptions &options)
      : Node("wheel_odometry", options), x_(0.0), y_(0.0), theta_(0.0),
        speed_(0.0), angular_speed_(0.0), prev_left_position_(0.0),
        prev_right_position_(0.0), prev_time_(this->now()),
        left_encoder_sub_(this, "/autodrive/roboracer_1/left_encoder"),
        right_encoder_sub_(this, "/autodrive/roboracer_1/right_encoder"),
        sync_(MySyncPolicy(20), left_encoder_sub_, right_encoder_sub_) {
    wheel_radius_ =
        this->declare_parameter<double>("wheel_radius", 0.0590); // meters
    wheel_base_ =
        this->declare_parameter<double>("wheel_base", 0.3240); // meters
    counts_per_rev_ =
        this->declare_parameter<double>("counts_per_rev",
                                        6.5); // encoder counts per revolution
    publish_tf_ =
        this->declare_parameter<bool>("publish_tf", false); // publish TF
    max_acceleration_ =
        this->declare_parameter<double>("max_acceleration", 4.0);

    // Register the callback with the synchronizer
    sync_.registerCallback(std::bind(&WheelOdometryNode::encoders_callback,
                                     this, std::placeholders::_1,
                                     std::placeholders::_2));

    // Publisher for odometry
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Subscriber for last command
    steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/autodrive/roboracer_1/steering", 1,
        std::bind(&WheelOdometryNode::steering_callback, this,
                  std::placeholders::_1));

    // TF Broadcaster
    tf_broadcaster_ = std::unique_ptr<tf2_ros::TransformBroadcaster>(
        new tf2_ros::TransformBroadcaster(*this));
  }

private:
  // Define the synchronization policy for ApproximateTime with two messages
  typedef mf::sync_policies::ApproximateTime<sensor_msgs::msg::JointState,
                                             sensor_msgs::msg::JointState>
      MySyncPolicy;

  void steering_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    last_steering_angle_ = msg->data;
    first_steering_msg_ = true;
  }

  // Callback function to process synchronized encoder messages
  void encoders_callback(
      const sensor_msgs::msg::JointState::ConstSharedPtr &left_msg,
      const sensor_msgs::msg::JointState::ConstSharedPtr &right_msg) {
    // Choose latest timestamp
    rclcpp::Time left_time(left_msg->header.stamp);
    rclcpp::Time right_time(right_msg->header.stamp);
    current_time_ = (left_time < right_time) ? right_msg->header.stamp
                                             : left_msg->header.stamp;

    if (!first_steering_msg_) {
      publish_current_pose();
      return;
    }

    // Update current encoder positions
    current_left_position_ = left_msg->position[0];
    current_right_position_ = right_msg->position[0];

    // Compute time difference since last update
    double delta_time = (current_time_ - prev_time_).seconds();

    if (delta_time <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
                  "Non-positive delta time. Skipping odometry update.");
      return;
    }

    // Calculate change in encoder positions
    double delta_left = current_left_position_ - prev_left_position_;
    double delta_right = current_right_position_ - prev_right_position_;

    // Update previous positions and time
    prev_left_position_ = current_left_position_;
    prev_right_position_ = current_right_position_;
    prev_time_ = current_time_;

    // Calculate speed for each wheel
    double speed_left = (2 * M_PI * wheel_radius_) *
                        (delta_left / counts_per_rev_) / delta_time;
    double speed_right = (2 * M_PI * wheel_radius_) *
                         (delta_right / counts_per_rev_) / delta_time;

    // Take minimum speed for both wheels
    double estimated_speed = (speed_left + speed_right) / 2.0;

    // Prevent outliers
    if (abs(speed_ - estimated_speed) < max_acceleration_) {
      speed_ = estimated_speed;
    }

    // Compute angular speed with bicycle motion model
    double angular_speed = 0.0;
    if (abs(last_steering_angle_) > 1e-3) {
      angular_speed = speed_ * tan(last_steering_angle_) / wheel_base_;
    }

    // Propagation
    x_ += speed_ * cos(theta_) * delta_time;
    y_ += speed_ * sin(theta_) * delta_time;
    theta_ += angular_speed * delta_time;
    theta_ = normalize_angle(theta_);

    publish_current_pose();
  }

  void publish_current_pose() {
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Set position
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    // Set orientation using TF2
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    q.normalize();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Setting velocities
    odom_msg.twist.twist.linear.x = speed_;
    odom_msg.twist.twist.angular.z = angular_speed_;

    // Publish odometry message
    odom_pub_->publish(odom_msg);

    // Create and send TF transform
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped odom_tf;
      odom_tf.header.stamp = current_time_;
      odom_tf.header.frame_id = "odom";
      odom_tf.child_frame_id = "base_link";

      odom_tf.transform.translation.x = x_;
      odom_tf.transform.translation.y = y_;
      odom_tf.transform.translation.z = 0.0;

      odom_tf.transform.rotation.x = odom_msg.pose.pose.orientation.x;
      odom_tf.transform.rotation.y = odom_msg.pose.pose.orientation.y;
      odom_tf.transform.rotation.z = odom_msg.pose.pose.orientation.z;
      odom_tf.transform.rotation.w = odom_msg.pose.pose.orientation.w;

      tf_broadcaster_->sendTransform(odom_tf);
    }
  }

  double normalize_angle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  // Parameters
  double wheel_radius_;
  double wheel_base_;
  double counts_per_rev_;
  bool publish_tf_;
  double max_acceleration_;

  // Robot Pose
  double x_;
  double y_;
  double theta_;
  double speed_;
  double angular_speed_;

  // Encoder Positions
  double current_left_position_;
  double current_right_position_;
  double prev_left_position_;
  double prev_right_position_;

  // Steering
  double last_steering_angle_;
  bool first_steering_msg_ = false;

  // Timestamps
  rclcpp::Time current_time_;
  rclcpp::Time prev_time_;

  // Message Filters Subscribers
  mf::Subscriber<sensor_msgs::msg::JointState> left_encoder_sub_;
  mf::Subscriber<sensor_msgs::msg::JointState> right_encoder_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;

  // Synchronizer with ApproximateTime policy
  mf::Synchronizer<MySyncPolicy> sync_;

  // Publisher and TF Broadcaster
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
}; // namespace autodrive_bridge

#include "rclcpp_components/register_node_macro.hpp" // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(autodrive_bridge::WheelOdometryNode)