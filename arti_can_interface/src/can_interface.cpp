#include <arti_can_interface/can_interface.h>
#include <algorithm>
#include <arti_can_interface/can_message_formatter.h>
#include <arti_can_interface/lely_can_interface.h>
#include <boost/make_shared.hpp>
#include <ros/console.h>
#include <utility>

namespace arti_can_interface
{
CanInterfacePtr CanInterface::createCanInterface(const std::string& device_name)
{
  return CanInterfacePtr(new LelyCanInterface(device_name));
}

void CanInterface::sendMessage(const arti_can_msgs::CanMessageConstPtr& can_message)
{
  sendMessage(*can_message);
}

void CanInterface::sendMessage(uint32_t id, std::vector<uint8_t> data)
{
  sendMessage(id, false, false, std::move(data));
}

void CanInterface::sendMessage(uint32_t id, bool rtr, bool extended_id, std::vector<uint8_t> data)
{
  arti_can_msgs::CanMessagePtr can_message = createMessage(id, rtr, extended_id);
  can_message->data = std::move(data);
  sendMessage(can_message);
}

void CanInterface::sendMessage(
  uint32_t id, bool rtr, bool extended_id, const std::vector<uint8_t>& data, size_t start_index, size_t size)
{
  arti_can_msgs::CanMessagePtr can_message = createMessage(id, rtr, extended_id);
  if (start_index < data.size())
  {
    if ((start_index + size) <= data.size())
    {
      can_message->data.assign(data.begin() + start_index,
                               data.begin() + start_index + size);
    }
    else
    {
      can_message->data.assign(data.begin() + start_index, data.end());
    }
  }
  sendMessage(can_message);
}

void CanInterface::sendMessage(uint32_t id, bool rtr, bool extended_id, const uint8_t* data, size_t size)
{
  arti_can_msgs::CanMessagePtr can_message = createMessage(id, rtr, extended_id);
  can_message->data.assign(data, data + size);
  sendMessage(can_message);
}

void CanInterface::subscribe(const std::vector<uint32_t>& ids, const MessageCallback& callback)
{
  for (const auto id : ids)
  {
    subscribe(id, callback);
  }
}

void CanInterface::subscribeToAll(MessageCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  callbacks_for_all_.emplace_back(std::move(callback));
}

void CanInterface::subscribe(uint32_t id, MessageCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  callbacks_.emplace(id, std::move(callback));
}

arti_can_msgs::CanMessagePtr CanInterface::createMessage(uint32_t id, bool rtr, bool extended_id)
{
  arti_can_msgs::CanMessagePtr can_message = boost::make_shared<arti_can_msgs::CanMessage>();
  can_message->id = id;
  can_message->rtr = rtr ? 1 : 0;
  can_message->extended_id = extended_id ? 1 : 0;
  return can_message;
}

void CanInterface::checkMessage(const arti_can_msgs::CanMessageConstPtr& can_message)
{
  checkMessage(*can_message);
}

void CanInterface::dispatchReceivedCanMessage(const arti_can_msgs::CanMessageConstPtr& can_message)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);

  const auto callback_range = callbacks_.equal_range(can_message->id);
  for (auto callback_it = callback_range.first; callback_it != callback_range.second; ++callback_it)
  {
    callMessageCallbackSafely(callback_it->second, can_message);
  }

  for (const auto& callback : callbacks_for_all_)
  {
    callMessageCallbackSafely(callback, can_message);
  }
}

void CanInterface::callMessageCallbackSafely(
  const MessageCallback& message_callback, const arti_can_msgs::CanMessageConstPtr& can_message)
{
  if (!message_callback)
  {
    ROS_WARN_STREAM("found invalid CAN message callback");
  }
  else
  {
    try
    {
      message_callback(can_message);
    }
    catch (const std::exception& ex)
    {
      ROS_WARN_STREAM("exception in CAN message callback: " << ex.what());
    }
    catch (...)
    {
      ROS_WARN_STREAM("exception in CAN message callback");
    }
  }
}

}
