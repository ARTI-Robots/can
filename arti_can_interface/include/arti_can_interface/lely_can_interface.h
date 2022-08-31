#ifndef ARTI_CAN_INTERFACE_LELY_CAN_INTERFACE_H
#define ARTI_CAN_INTERFACE_LELY_CAN_INTERFACE_H

#include <arti_can_interface/can_interface.h>
#include <atomic>
#include <condition_variable>
#include <lely/coapp/slave.hpp>
#include <lely/can/msg.h>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>
#include <thread>

namespace arti_can_interface
{
/**
 * \brief implementation of the can interface using the lely library
 */
class LelyCanInterface : public CanInterface
{
public:
  explicit LelyCanInterface(const std::string& device_name);
  ~LelyCanInterface() override;

  void sendMessage(const arti_can_msgs::CanMessage& can_message) override;
  void checkMessage(const arti_can_msgs::CanMessage& can_message) override;

protected:
  static void doCheckMessage(const arti_can_msgs::CanMessage& can_message);

  void runEventLoop();
  void runReadThread();
  void notifyReadResult(int can_receive_result, const std::error_code& ec);

  static can_msg convertToInternalCanMsg(const arti_can_msgs::CanMessage& can_message);
  static arti_can_msgs::CanMessageConstPtr convertToCanMessage(const can_msg& internal_can_message);
  static void convertToCanMessage(const can_msg& internal_can_message, arti_can_msgs::CanMessage& can_message);

  std::thread read_thread_;
  std::atomic<bool> should_read_{true};
  std::atomic<bool> should_write_{true};

  lely::io::IoGuard io_guard_;
  lely::io::Context ctx_;
  lely::io::Poll poll_;
  lely::ev::Loop event_loop_;
  lely::io::CanController can_controller_;
  lely::io::CanChannel can_channel_;
  std::thread event_loop_thread_;

  std::mutex notify_read_mutex_;
  std::condition_variable notify_read_condition_;
  bool got_frame_{false};
};
}

#endif //ARTI_CAN_INTERFACE_LELY_CAN_INTERFACE_H
