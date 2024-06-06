#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <chrono>
#include <thread>

class RobotChase : public rclcpp::Node {
public:
  RobotChase() : Node("robot_chase_node") {

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    std::string robot_chaser = "rick";
    std::string robot_chasee = "morty";
    // Publishers and Subscribers
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        robot_chaser + "/cmd_vel", 10);

    // Parameters
    kp_distance_ = 0.1;             // Proportional gain for distance
    kp_yaw_ = 0.4;                  // Proportional gain for yaw
    min_linear_velocity_ = 0.3;     // Minimum linear velocity
    min_angular_velocity_ = 0.3;    // Minimum angular velocity
    min_distance_threshold_ = 0.60; // Minimum distance threshold
    stopping_distance_threshold_ = 0.36;

    // Timer
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&RobotChase::update_command, this));
  }

private:
  void update_command() {
    auto transform_opt = get_transform("morty/base_link", "rick/base_link");
    if (!transform_opt) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform.");
      return;
    }

    auto &transform = *transform_opt;
    double error_distance = sqrt(pow(transform.transform.translation.x, 2) +
                                 pow(transform.transform.translation.y, 2));
    // Print the error distance for debugging
    RCLCPP_INFO(this->get_logger(), "Error Distance: %f", error_distance);

    // Assuming a flat plane, yaw error is derived from quaternion to yaw
    // conversion
    tf2::Quaternion q(
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Print roll, pitch, and yaw values for debugging
    RCLCPP_INFO(this->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f", roll, pitch,
                yaw);

    double error_yaw = atan2(transform.transform.translation.y,
                             transform.transform.translation.x);
    // Print the yaw error for debugging
    RCLCPP_INFO(this->get_logger(), "Error Yaw: %f", error_yaw);

    // Create and publish the Twist message
    geometry_msgs::msg::Twist cmd_vel;

    if (error_distance <= stopping_distance_threshold_) {
      // Stop the robot if it's closer than the stopping distance threshold
      RCLCPP_INFO(this->get_logger(), "Morty was caught by Rick!!!");
      RCLCPP_INFO(this->get_logger(), "Morty was caught by Rick!!!");
      RCLCPP_INFO(this->get_logger(), "Morty was caught by Rick!!!");
      RCLCPP_INFO(this->get_logger(), "Morty was caught by Rick!!!");
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
    } else {
      // Compute velocities based on the error distance and yaw
      cmd_vel.linear.x = kp_distance_ * error_distance;
      cmd_vel.angular.z = kp_yaw_ * error_yaw;

      // Ensure minimum velocities are considered
      if (error_distance >= min_distance_threshold_) {
        // Make sure linear velocity is at least the minimum linear velocity
        cmd_vel.linear.x = std::copysign(
            std::max(std::abs(cmd_vel.linear.x), min_linear_velocity_),
            cmd_vel.linear.x);

        // Make sure angular velocity is at least the minimum angular velocity
        cmd_vel.angular.z = std::copysign(
            std::max(std::abs(cmd_vel.angular.z), min_angular_velocity_),
            cmd_vel.angular.z);
      }
    }

    velocity_publisher_->publish(cmd_vel);

    // Print the command velocities being published
    RCLCPP_INFO(this->get_logger(), "Publishing Linear X: %f, Angular Z: %f",
                cmd_vel.linear.x, cmd_vel.angular.z);
  }

  std::optional<geometry_msgs::msg::TransformStamped>
  get_transform(const std::string &from_frame, const std::string &to_frame) {
    try {
      return tf_buffer_->lookupTransform(to_frame, from_frame,
                                         tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "TF2 error: %s", ex.what());
      return std::nullopt;
    }
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double kp_distance_, kp_yaw_;
  double min_linear_velocity_, min_angular_velocity_, min_distance_threshold_,
      stopping_distance_threshold_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotChase>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}