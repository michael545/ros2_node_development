#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "basic_pub_sub/subscriber_core.hpp"

class Listener : public rclcpp::Node
{
public:
  Listener()
  : Node("listener"), subscriber_core_()
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&Listener::topic_callback, this, std::placeholders::_1));

    subscriber_core_.set_callback(
      [this](const std::string & msg) {
        RCLCPP_INFO(this->get_logger(), "Processed: '%s'", msg.c_str());
      });
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    subscriber_core_.process_message(msg->data);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  mutable basic_pub_sub::SubscriberCore subscriber_core_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}