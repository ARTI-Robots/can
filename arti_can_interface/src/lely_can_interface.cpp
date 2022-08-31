#include <arti_can_interface/lely_can_interface.h>
#include <arti_can_interface/can_message_formatter.h>
#include <ros/console.h>

namespace arti_can_interface
{
LelyCanInterface::LelyCanInterface(const std::string& device_name)
  : poll_(ctx_), event_loop_(poll_.get_poll()), can_controller_(device_name.c_str()),
    can_channel_(poll_, event_loop_.get_executor())
{
  can_channel_.open(can_controller_);

  event_loop_thread_ = std::thread(&LelyCanInterface::runEventLoop, this);
  read_thread_ = std::thread(&LelyCanInterface::runReadThread, this);
}

LelyCanInterface::~LelyCanInterface()
{
  should_read_ = false;
  should_write_ = false;

  {
    std::unique_lock<std::mutex> lock(notify_read_mutex_);
    notify_read_condition_.notify_all();
  }

  ctx_.shutdown();
  event_loop_.stop();

  if (read_thread_.joinable())
  {
    read_thread_.join();
  }

  if (event_loop_thread_.joinable())
  {
    event_loop_thread_.join();
  }
}

void LelyCanInterface::sendMessage(const arti_can_msgs::CanMessage& can_message)
{
  can_msg internal_can_message = convertToInternalCanMsg(can_message);
  try
  {
    ROS_DEBUG_STREAM_NAMED("CanInterface", "sendMessage");
    can_channel_.write(internal_can_message);
  }
  catch (const std::exception& exp)
  {
    ROS_ERROR_STREAM("an error occurred when writing to can channel: " << exp.what());
    throw exp;
  }
}

void LelyCanInterface::checkMessage(const arti_can_msgs::CanMessage& can_message)
{
  doCheckMessage(can_message);
}

void LelyCanInterface::doCheckMessage(const arti_can_msgs::CanMessage& can_message)
{
  if (can_message.id >= (1U << 29U) || (!can_message.extended_id && can_message.id >= (1U << 11U)))
  {
    throw std::invalid_argument("CAN message ID is out of bounds");
  }
  if (can_message.data.size() > CAN_MSG_MAX_LEN)
  {
    throw std::invalid_argument("CAN message data is too long (" + std::to_string(can_message.data.size()) + " > "
                                + std::to_string(CAN_MSG_MAX_LEN) + ")");
  }
}

void LelyCanInterface::runEventLoop()
{
  while (should_write_)
  {
    try
    {
      event_loop_.get_executor().on_task_init();
      event_loop_.run();
      event_loop_.get_executor().on_task_fini();
    }
    catch (const std::exception& exp)
    {
      ROS_ERROR_STREAM("an error occurred in the event loop: " << exp.what());
      throw exp;
    }
  }
}

void LelyCanInterface::runReadThread()
{
  while (should_read_)
  {
    ROS_DEBUG_STREAM_NAMED("CanInterface", "runReadThread 1");
    can_msg internal_can_message = CAN_MSG_INIT;
    can_err internal_can_error = CAN_ERR_INIT;
    try
    {
      ROS_DEBUG_STREAM_NAMED("CanInterface", "==========================================");
      ROS_DEBUG_STREAM_NAMED("CanInterface", "read frame async");
      std::unique_lock<std::mutex> lock(notify_read_mutex_);
      got_frame_ = false;
      //can_channel_.read(&internal_can_message, &internal_can_error);
      can_channel_.submit_read(&internal_can_message, &internal_can_error, nullptr,
                               std::bind(&LelyCanInterface::notifyReadResult, this, std::placeholders::_1,
                                         std::placeholders::_2));
      ROS_DEBUG_STREAM_NAMED("CanInterface", "wait for result frame async");
      while (!got_frame_ && should_read_)
      {
        notify_read_condition_.wait(lock);
      }
      if (!should_read_)
      {
        break;
      }

      ROS_DEBUG_STREAM_NAMED("CanInterface", "don reading frame");
      ROS_DEBUG_STREAM_NAMED("CanInterface", "******************************************");
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR_STREAM_NAMED("CanInterface", "an error occurred in the read loop, exiting read loop: " << ex.what());
      throw ex;
    }
    ROS_DEBUG_STREAM_NAMED("CanInterface", "runReadThread 2");
    ROS_DEBUG_STREAM_NAMED("CanInterface",
                           "internal_can_message: id: " << static_cast<int>(internal_can_message.id) << " flags: "
                                                        << static_cast<int>(internal_can_message.flags) << " len: "
                                                        << static_cast<int>(internal_can_message.len) << " data:");
    for (size_t i = 0; i < internal_can_message.len; ++i)
    {
      ROS_DEBUG_STREAM_NAMED("CanInterface", "byte[" << i << "]" << static_cast<int>(internal_can_message.data[i]));
    }

    const arti_can_msgs::CanMessageConstPtr can_message = convertToCanMessage(internal_can_message);
    ROS_DEBUG_STREAM_NAMED("CanInterface", "received CAN message: " << CanMessageFormatter(*can_message));

    dispatchReceivedCanMessage(can_message);
  }
}

void LelyCanInterface::notifyReadResult(int can_receive_result, const std::error_code& ec)
{
  std::unique_lock<std::mutex> lock(notify_read_mutex_);

  ROS_DEBUG_STREAM_NAMED("CanInterface",
                         "received a frame with receiving info: " << can_receive_result << " and error code: " << ec);

  got_frame_ = true;
  notify_read_condition_.notify_all();
}

can_msg LelyCanInterface::convertToInternalCanMsg(const arti_can_msgs::CanMessage& can_message)
{
  doCheckMessage(can_message);

  can_msg internal_can_message = CAN_MSG_INIT;
  internal_can_message.id = can_message.id;
  internal_can_message.len = can_message.data.size();
  std::copy(can_message.data.begin(), can_message.data.end(), &internal_can_message.data[0]);

  internal_can_message.flags = 0;
  if (can_message.extended_id)
  {
    internal_can_message.flags |= CAN_FLAG_IDE;
  }

  if (can_message.rtr)
  {
    internal_can_message.flags |= CAN_FLAG_RTR;
  }

  if (can_message.extended_data_length)
  {
    internal_can_message.flags |= CAN_FLAG_EDL;
  }

  if (can_message.bit_rate_switch)
  {
    internal_can_message.flags |= CAN_FLAG_BRS;
  }

  if (can_message.error_state_indicator)
  {
    internal_can_message.flags |= CAN_FLAG_ESI;
  }

  return internal_can_message;
}

arti_can_msgs::CanMessageConstPtr LelyCanInterface::convertToCanMessage(const can_msg& internal_can_message)
{
  arti_can_msgs::CanMessagePtr can_message(new arti_can_msgs::CanMessage);
  convertToCanMessage(internal_can_message, *can_message);
  return can_message;
}

void LelyCanInterface::convertToCanMessage(const can_msg& internal_can_message, arti_can_msgs::CanMessage& can_message)
{
  can_message.id = internal_can_message.id;
  can_message.data.assign(&internal_can_message.data[0], &internal_can_message.data[internal_can_message.len]);
  can_message.extended_id = (internal_can_message.flags & CAN_FLAG_IDE ? 1 : 0);
  can_message.rtr = (internal_can_message.flags & CAN_FLAG_RTR ? 1 : 0);
  can_message.extended_data_length = (internal_can_message.flags & CAN_FLAG_EDL ? 1 : 0);
  can_message.bit_rate_switch = (internal_can_message.flags & CAN_FLAG_BRS ? 1 : 0);
  can_message.error_state_indicator = (internal_can_message.flags & CAN_FLAG_ESI ? 1 : 0);
}
}
