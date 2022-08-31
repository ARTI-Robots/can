#ifndef ARTI_CAN_INTERFACE_CAN_INTERFACE_NODE_H
#define ARTI_CAN_INTERFACE_CAN_INTERFACE_NODE_H

#include <arti_can_interface/can_interface.h>
#include <fstream>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/timer.h>
#include <string>

namespace arti_can_interface
{
class CanInterfaceNode
{
public:
  explicit CanInterfaceNode(const ros::NodeHandle& node_handle);

private:
  static const char DIRECTION_RX[];
  static const char DIRECTION_TX[];

  void transmitCanMessage(const arti_can_msgs::CanMessageConstPtr& can_message);
  void processReceivedCanMessage(const arti_can_msgs::CanMessageConstPtr& can_message);
  void dump(const char* direction, const arti_can_msgs::CanMessage& can_message);
  void processNextCommand(const ros::TimerEvent&);
  void processCommand(const std::string& command);

  ros::NodeHandle node_handle_;
  std::string device_name_;
  std::string dump_file_name_;
  std::string command_file_name_;
  char separator_;
  ros::Duration command_interval_;

  ros::Subscriber can_message_subscriber_;
  ros::Publisher can_message_publisher_;
  ros::Timer command_timer_;

  CanInterfacePtr can_interface_;
  std::ofstream dump_file_stream_;
  std::ifstream command_file_stream_;
};
}

#endif //ARTI_CAN_INTERFACE_CAN_INTERFACE_NODE_H
