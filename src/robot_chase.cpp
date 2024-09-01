#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

class RobotChase : public rclcpp::Node {
public:
  RobotChase()
      : Node("robot_chase_node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    // Publisher for Rick's velocity commands on the /rick/cmd_vel topic
    velocity_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/rick/cmd_vel", 10);

    // Introducing a small delay to ensure the TF listener has time to start
    // receiving data
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Timer to periodically call the update function every 100 milliseconds
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                     std::bind(&RobotChase::update, this));

    // Control gains as specified
    kp_yaw_ = 2.00;
    kp_distance_ = 0.15;

    // Initial debug statement
    RCLCPP_INFO(this->get_logger(), "RobotChase node initialized.");
  }

private:
  void update() {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      // Attempt to get the latest transform from Morty's base_link to Rick's
      // base_link
      transform_stamped = tf_buffer_.lookupTransform(
          "rick/base_link", "morty/base_link", tf2::TimePointZero);
      RCLCPP_INFO(this->get_logger(), "Transform received: x: %f, y: %f, z: %f",
                  transform_stamped.transform.translation.x,
                  transform_stamped.transform.translation.y,
                  transform_stamped.transform.translation.z);
    } catch (tf2::TransformException &ex) {
      // If the transform is not available, log a warning and exit the function
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    // Relative position of Morty to Rick
    double dx = transform_stamped.transform.translation.x;
    double dy = transform_stamped.transform.translation.y;

    RCLCPP_INFO(this->get_logger(), "Relative position: dx: %f, dy: %f", dx,
                dy);

    // Calculate the distance error between Rick and Morty
    double error_distance = sqrt(dx * dx + dy * dy);
    RCLCPP_INFO(this->get_logger(), "Error Distance: %f", error_distance);

    // Calculate the angular error (yaw) based on the relative position of Morty
    // to Rick
    double error_yaw = atan2(dy, dx);

    // Normalize yaw error to the range [-pi, pi]
    if (error_yaw > M_PI) {
      error_yaw -= 2 * M_PI;
    } else if (error_yaw < -M_PI) {
      error_yaw += 2 * M_PI;
    }

    RCLCPP_INFO(this->get_logger(), "Error Yaw (radians): %f", error_yaw);
    RCLCPP_INFO(this->get_logger(), "Error Yaw (degrees): %f",
                error_yaw * 180.0 / M_PI);

    // Calculate the linear and angular velocity commands using proportional
    // control
    double linear_velocity = kp_distance_ * error_distance;
    double angular_velocity = kp_yaw_ * error_yaw;

    RCLCPP_INFO(this->get_logger(), "Calculated linear velocity: %f",
                linear_velocity);
    RCLCPP_INFO(this->get_logger(), "Calculated angular velocity: %f",
                angular_velocity);

    // Create a Twist message to send the velocity commands to Rick
    double max_linear_velocity = 2.0;  // Max linear speed
    double max_angular_velocity = 2.0; // Max angular speed

    // Distance threshold to prevent collision
    double stop_threshold = 0.60; // Stop if closer than 0.2 meters

    auto twist_msg = geometry_msgs::msg::Twist();

    if (error_distance <= stop_threshold) {

      // Calculate a deceleration factor based on how close the robot is to the
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;

      velocity_publisher_->publish(twist_msg);
      RCLCPP_INFO(this->get_logger(), "Stopping: Too close to target.");
      RCLCPP_INFO(this->get_logger(),
                  "Twist message published: linear.x = %f, angular.z = %f",
                  twist_msg.linear.x, twist_msg.angular.z);

    } else if (error_distance > stop_threshold) {

      twist_msg.linear.x = std::max(
          std::min(linear_velocity, max_linear_velocity), -max_linear_velocity);
      twist_msg.angular.z =
          std::max(std::min(angular_velocity, max_angular_velocity),
                   -max_angular_velocity);
      velocity_publisher_->publish(twist_msg);

      RCLCPP_INFO(this->get_logger(),
                  "Twist message published: linear.x = %f, angular.z = %f",
                  twist_msg.linear.x, twist_msg.angular.z);
    }
  }

  // Member variables
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Control gains as specified
  double kp_yaw_;
  double kp_distance_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv); // Initialize the ROS2 client library
  auto node = std::make_shared<RobotChase>(); // Create an instance of the
                                              // RobotChase node
  rclcpp::spin(node); // Keep the node running, processing callbacks and timers
  rclcpp::shutdown(); // Shutdown the ROS2 client library when done
  return 0;
}
