#include "basic_pub_sub/publisher_core.hpp"

namespace basic_pub_sub
{

PublisherCore::PublisherCore()
: base_message_("Hello World"),
  count_(0)
{
}

void PublisherCore::set_message(const std::string & msg)
{
  base_message_ = msg;
}

std::string PublisherCore::get_next_message()
{
  return base_message_ + " " + std::to_string(count_++);
}

}  // namespace basic_pub_sub