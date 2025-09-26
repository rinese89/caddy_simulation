#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_msgs/msg/tf_message.hpp"

class OdomToTFBridge : public rclcpp::Node
{
public:
  OdomToTFBridge() : Node("odom2tf_broadcaster")
  {
    // Crear el broadcaster de TF
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Suscribirse al tópico de transformadas del controlador
    subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/ackermann_steering_controller/tf_odometry", 10,
      std::bind(&OdomToTFBridge::callback, this, std::placeholders::_1));
  }

private:
 void callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  for (auto transform : msg->transforms)
  {    
      //transform.child_frame_id = "base_footprint";
    
    tf_broadcaster_->sendTransform(transform);
  }
}

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomToTFBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
