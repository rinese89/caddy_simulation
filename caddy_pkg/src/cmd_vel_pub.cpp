#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

class AckermannPublisher : public rclcpp::Node {
public:
  AckermannPublisher() : Node("cmd_vel_pub") {
    this->declare_parameter("publish_rate", 10.0);
    double publish_rate = this->get_parameter("publish_rate").as_double();

    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/ackermann_steering_controller/reference", 10);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate),
        std::bind(&AckermannPublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Ackermann TwistStamped publisher started.");
  }

private:
  void timer_callback() {
    auto msg = geometry_msgs::msg::TwistStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = 1.0;
    msg.twist.angular.z = 0.2;
    publisher_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannPublisher>());
  rclcpp::shutdown();
  return 0;
}
