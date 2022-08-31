#include <arti_can_interface/can_message_formatter.h>

namespace arti_can_interface
{
CanMessageFormatter::CanMessageFormatter(const arti_can_msgs::CanMessage& can_message)
  : can_message_(can_message)
{
}

void CanMessageFormatter::format(std::ostream& out) const
{
  out << std::hex << std::noshowbase << std::right << std::setfill('0');  // Set format flags
  out << "id: 0x" << std::setw(can_message_.extended_id ? 8 : 3) << can_message_.id << ", data: [";
  for (std::size_t i = 0; i < can_message_.data.size(); ++i)
  {
    if (i != 0)
    {
      out << ", ";
    }
    out << "0x" << std::setw(2) << static_cast<unsigned int>(can_message_.data[i]);
  }
  out << "]";
  out << std::dec << std::setfill(' ');  // Reset format flags to sensible values
}

}

std::ostream& operator<<(std::ostream& out, const arti_can_interface::CanMessageFormatter& can_message_formatter)
{
  can_message_formatter.format(out);
  return out;
}
