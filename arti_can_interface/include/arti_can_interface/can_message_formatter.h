#ifndef ARTI_CAN_INTERFACE_CAN_MESSAGE_FORMATTER_H
#define ARTI_CAN_INTERFACE_CAN_MESSAGE_FORMATTER_H

#include <arti_can_msgs/CanMessage.h>
#include <iosfwd>

namespace arti_can_interface
{
/**
 * formatter class allowing the can message to be printed on a out-stream in a human readable form.
 * This class can also be send into a stream to allow formatted output.
 */
class CanMessageFormatter
{
public:
  /**
   * constructor with the message to format
   * \param can_message to print in a formated way
   */
  explicit CanMessageFormatter(const arti_can_msgs::CanMessage& can_message);

  /**
   * function to print the formatted can message on the out-stream
   * \param out stream to print the message to
   */
  void format(std::ostream& out) const;

protected:
  const arti_can_msgs::CanMessage& can_message_;
};
}

/**
 * overloaded stream function to print formatted message on an out-stream
 * \param out stream to print the message to
 * \param can_message_formatter formatter class doing the actually formatting
 * \return out stream with the printed message for further processing
 */
std::ostream& operator<<(std::ostream& out, const arti_can_interface::CanMessageFormatter& can_message_formatter);

#endif //ARTI_CAN_INTERFACE_CAN_MESSAGE_FORMATTER_H
