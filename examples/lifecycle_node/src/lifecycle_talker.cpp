#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleTalker()
  : LifecycleNode("lifecycle_talker"), count_(0)
  {
    RCLCPP_INFO(get_logger(), "LifecycleTalker created");
  }

  ~LifecycleTalker()
  {
    RCLCPP_INFO(get_logger(), "LifecycleTalker destroyed");
  }

  LifecycleNode::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Configuring...");

    // Create publisher
    publisher_ = create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);

    // Create timer (but don't start yet)
    timer_ = create_wall_timer(1s, std::bind(&LifecycleTalker::publish_message, this));

    RCLCPP_INFO(get_logger(), "Configuration complete");
    return LifecycleNode::CallbackReturn::SUCCESS;
  }

  LifecycleNode::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Activating...");

    // Activate publisher
    publisher_->on_activate();

    RCLCPP_INFO(get_logger(), "Activation complete");
    return LifecycleNode::CallbackReturn::SUCCESS;
  }

  LifecycleNode::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Deactivating...");

    // Deactivate publisher
    publisher_->on_deactivate();

    RCLCPP_INFO(get_logger(), "Deactivation complete");
    return LifecycleNode::CallbackReturn::SUCCESS;
  }

  LifecycleNode::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up...");

    // Destroy resources
    publisher_.reset();
    timer_.reset();

    RCLCPP_INFO(get_logger(), "Cleanup complete");
    return LifecycleNode::CallbackReturn::SUCCESS;
  }

  LifecycleNode::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Shutting down...");

    // Cleanup resources
    publisher_.reset();
    timer_.reset();

    RCLCPP_INFO(get_logger(), "Shutdown complete");
    return LifecycleNode::CallbackReturn::SUCCESS;
  }

private:
  void publish_message()
  {
    auto message = std_msgs::msg::String();
    message.data = "Lifecycle message " + std::to_string(count_++);
    RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LifecycleTalker>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();
  return 0;
}