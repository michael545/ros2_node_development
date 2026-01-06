#ifndef BASIC_PUB_SUB__SUBSCRIBER_CORE_HPP_
#define BASIC_PUB_SUB__SUBSCRIBER_CORE_HPP_

#include <string>
#include <functional>

namespace basic_pub_sub
{

class SubscriberCore
{
public:
  SubscriberCore();
  ~SubscriberCore() = default;

  using MessageCallback = std::function<void(const std::string &)>;

  void set_callback(MessageCallback callback);
  void process_message(const std::string & message);

private:
  MessageCallback callback_;
};

}  // namespace basic_pub_sub

#endif  // BASIC_PUB_SUB__SUBSCRIBER_CORE_HPP_