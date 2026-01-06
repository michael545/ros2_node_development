#ifndef BASIC_PUB_SUB__PUBLISHER_CORE_HPP_
#define BASIC_PUB_SUB__PUBLISHER_CORE_HPP_

#include <string>
#include <memory>
#include <chrono>

namespace basic_pub_sub
{

class PublisherCore
{
public:
  PublisherCore();
  ~PublisherCore() = default;

  void set_message(const std::string & msg);
  std::string get_next_message();

private:
  std::string base_message_;
  int count_;
};

}  // namespace basic_pub_sub

#endif  // BASIC_PUB_SUB__PUBLISHER_CORE_HPP_