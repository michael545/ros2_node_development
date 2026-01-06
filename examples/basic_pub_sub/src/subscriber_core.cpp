#include "basic_pub_sub/subscriber_core.hpp"
#include <iostream>

namespace basic_pub_sub
{

SubscriberCore::SubscriberCore()
{
}

void SubscriberCore::set_callback(MessageCallback callback)
{
  callback_ = callback;
}

void SubscriberCore::process_message(const std::string & message)
{
  if (callback_) {
    callback_(message);
  } else {
    std::cout << "Received: " << message << std::endl;
  }
}

}  // namespace basic_pub_sub