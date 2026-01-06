#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParameterNode : public rclcpp::Node
{
public:
  ParameterNode()
  : Node("parameter_node")
  {
    // Declare parameters with default values
    this->declare_parameter("publish_rate", 1.0);
    this->declare_parameter("message", "Default message");
    this->declare_parameter("enable_logging", true);

    // Create a timer to periodically check parameters
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&ParameterNode::timer_callback, this));

    // Create parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ParameterNode::parameter_callback, this, std::placeholders::_1));
  }

private:
  void timer_callback()
  {
    // Get current parameter values
    double publish_rate = this->get_parameter("publish_rate").as_double();
    std::string message = this->get_parameter("message").as_string();
    bool enable_logging = this->get_parameter("enable_logging").as_bool();

    if (enable_logging) {
      RCLCPP_INFO(this->get_logger(), "Rate: %.2f, Message: %s", publish_rate, message.c_str());
    }
  }

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "publish_rate") {
        if (param.as_double() <= 0.0) {
          result.successful = false;
          result.reason = "publish_rate must be positive";
        }
      }
    }

    return result;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParameterNode>());
  rclcpp::shutdown();
  return 0;
}