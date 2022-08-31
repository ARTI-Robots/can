#include <arti_can_interface/can_interface_node.h>
#include <arti_can_interface/can_message_formatter.h>
#include <ctime>

namespace arti_can_interface
{
const char CanInterfaceNode::DIRECTION_RX[] = "rx";
const char CanInterfaceNode::DIRECTION_TX[] = "tx";

CanInterfaceNode::CanInterfaceNode(const ros::NodeHandle& node_handle)
  : node_handle_(node_handle), device_name_(node_handle_.param<std::string>("device_name", "")),
    dump_file_name_(node_handle_.param<std::string>("dump_file_name", "")),
    command_file_name_(node_handle_.param<std::string>("command_file_name", "")),
    separator_(node_handle_.param<std::string>("separator", ";").at(0)),
    command_interval_(node_handle_.param<double>("command_interval", 0.1)),
    can_message_subscriber_(node_handle_.subscribe("tx", 10, &CanInterfaceNode::transmitCanMessage, this)),
    can_message_publisher_(node_handle_.advertise<arti_can_msgs::CanMessage>("rx", 10)),
    can_interface_(CanInterface::createCanInterface(device_name_))
{
  ROS_INFO_STREAM("using connection to device: '" << device_name_ << "'");

  if (!dump_file_name_.empty())
  {
    dump_file_stream_.open(dump_file_name_, std::ios_base::out | std::ios_base::app);
    if (dump_file_stream_)
    {
      ROS_INFO_STREAM("appending to file '" << dump_file_name_ << "'");
      char datetime[32] = "";
      const time_t now = time(nullptr);
      strftime(datetime, sizeof(datetime), "%FT%T%z", localtime(&now));
      dump_file_stream_ << "# Dump started " << datetime << std::endl;
      dump_file_stream_ << std::hex << std::noshowbase << std::right << std::setfill('0');  // Set format flags
    }
    else
    {
      ROS_ERROR_STREAM("file '" << dump_file_name_ << "' could not be opened for appending");
    }
  }

  if (!command_file_name_.empty())
  {
    command_file_stream_.open(command_file_name_);
    if (command_file_stream_)
    {
      ROS_INFO_STREAM("reading from file '" << command_file_name_ << "'");

      command_timer_ = node_handle_.createTimer(command_interval_, &CanInterfaceNode::processNextCommand, this);
    }
    else
    {
      ROS_ERROR_STREAM("file '" << command_file_name_ << "' could not be opened for reading");
    }
  }

  can_interface_->subscribeToAll(std::bind(&CanInterfaceNode::processReceivedCanMessage, this, std::placeholders::_1));
}

void CanInterfaceNode::transmitCanMessage(const arti_can_msgs::CanMessageConstPtr& can_message)
{
  ROS_INFO_STREAM("transmitting CAN message: " << CanMessageFormatter(*can_message));
  can_interface_->sendMessage(can_message);
  dump(DIRECTION_TX, *can_message);
}

void CanInterfaceNode::processReceivedCanMessage(const arti_can_msgs::CanMessageConstPtr& can_message)
{
  ROS_INFO_STREAM("received CAN message: " << CanMessageFormatter(*can_message));
  can_message_publisher_.publish(can_message);
  dump(DIRECTION_RX, *can_message);
}

void CanInterfaceNode::dump(const char* const direction, const arti_can_msgs::CanMessage& can_message)
{
  if (dump_file_stream_)
  {
    dump_file_stream_ << direction << separator_ << "0x" << std::setw(8) << can_message.id;
    for (const auto byte : can_message.data)
    {
      dump_file_stream_ << separator_ << "0x" << std::setw(2) << static_cast<unsigned int>(byte);
    }
    dump_file_stream_ << std::endl;
  }
}

void CanInterfaceNode::processNextCommand(const ros::TimerEvent& /*timer_event*/)
{
  std::string command;
  while (command_file_stream_ && std::getline(command_file_stream_, command))
  {
    if (!command.empty() && command.at(0) != '#')
    {
      processCommand(command);
      break;
    }
  }

  if (!command_file_stream_)
  {
    command_timer_.stop();
    if (command_file_stream_.eof())
    {
      ROS_INFO_STREAM("finished executing commands");
    }
    else
    {
      ROS_WARN_STREAM("error reading command file");
    }
  }
}

void CanInterfaceNode::processCommand(const std::string& command)
{
  arti_can_msgs::CanMessagePtr can_message(new arti_can_msgs::CanMessage);

  bool valid = true;
  std::istringstream command_stream(command);
  std::string item;
  std::string direction;
  for (std::size_t i = 0; std::getline(command_stream, item, separator_) && !item.empty(); ++i)
  {
    if (i == 0)
    {
      direction = std::move(item);
    }
    else
    {
      std::size_t end_index = 0;
      const unsigned long value = std::stoul(item, &end_index, 0);
      if (end_index != item.size())
      {
        ROS_WARN_STREAM("invalid line in command file: string cannot be parsed as a number: '" << item << "'");
        valid = false;
      }
      else if (i == 1)
      {
        if (value <= 0x7ffu)
        {
          can_message->id = value;
        }
        else
        {
          ROS_WARN_STREAM("invalid line in command file: ID (second value in line) must be between 0 and 0x7ff, but"
                          " is 0x" << std::hex << value);
          valid = false;
        }
      }
      else
      {
        if (value <= 0xffu)
        {
          can_message->data.emplace_back(value);
        }
        else
        {
          ROS_WARN_STREAM("invalid line in command file: data (third and following values in line) must be between 0"
                          " and 0xff, but is 0x" << std::hex << value);
          valid = false;
        }
      }
    }
  }

  if (valid)
  {
    if (direction == DIRECTION_RX)
    {
      // Fake reception:
      processReceivedCanMessage(can_message);
    }
    else if (direction == DIRECTION_TX)
    {
      transmitCanMessage(can_message);
    }
    else
    {
      ROS_WARN_STREAM("invalid line in command file: direction (first value in line) must be"
                      " either '" << DIRECTION_RX << "' or '" << DIRECTION_TX << "', but is '" << direction << "'");
    }
  }
}
}

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "can_interface");
    ros::NodeHandle node_handle("~");
    arti_can_interface::CanInterfaceNode node(node_handle);
    ros::spin();
  }
  catch (const std::exception& ex)
  {
    std::cerr << "Unhandled exception: " << ex.what() << std::endl;
    return 1;
  }
  catch (...)
  {
    std::cerr << "Unhandled exception" << std::endl;
    return 1;
  }
  return 0;
}