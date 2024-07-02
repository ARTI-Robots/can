# CAN

Interface for CAN based devices. This also includes interfaces for CANopen.

This repository contains the following packages:

- *arti_can_interface* contains the library to access the CAN more easily. The package also contains a node for CAN bus introspection.
- *arti_can_msgs* contains messages sent by the nodes to allow introspection of the CAN bus or sending messages to the CAN bus.


## Installation of the lely CANopen library

Installation instructions are taken from https://opensource.lely.com/canopen/docs/installation/.
The installation is performed from sources, as Ubuntu 16.04 does not provide the proper library packages:

```sh
sudo apt-get install \
    git build-essential automake libtool \
    libbluetooth-dev \
    valgrind \
    doxygen graphviz can-utils

cd ~/Downloads/
git clone -b v2.2.1 https://gitlab.com/lely_industries/lely-core.git
cd lely-core

autoreconf -i
mkdir build
cd build
../configure --disable-python

make -j8 -l8
make check
sudo make install
```


## Setting up a CAN interface

```sh
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 txqueuelen 1024
sudo ip link set up can0
```

Note: if the TX queue length is too small, opening the CAN interface with lely may result in an "operation not permitted" error.


## Setting up a virtual CAN interface

To test a CAN connection one can create a virtual CAN device:

```sh
sudo modprobe vcan

sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

ip link show vcan0
```

The `show` command should show something similar to:

```
3: vcan0: <NOARP,UP,LOWER_UP> mtu 72 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000
    link/can
```

There is also a script for this purpose:

```sh
rosrun arti_can_interface setup_vcan.sh vcan0
```

To delete a virtual CAN device, run:

```sh
sudo ip link set down vcan0
sudo ip link delete dev vcan0
```

or

```sh
rosrun arti_can_interface delete_vcan.sh vcan0
```


## Testing CAN interfaces

The output of a CAN message can be shown by dumping the device as follows:

```sh
candump vcan0
```

A CAN message can be sent using the following command, giving the CAN device, ID, and data bytes:

```sh
cansend vcan0 18c#017f
```

To setup two CAN devices talking to each other use the following:

```sh
sudo modprobe can-gw

sudo cangw -A -s vcan0 -d vcan1 -e
sudo cangw -A -s vcan1 -d vcan0 -e
```

To create two virtual CAN devices talking to each other use:

```sh
rosrun arti_can_interface setup_connected_vcans.sh vcan0 vcan1
```

The CAN interface node can also be used with a dump file and a command file:

```sh
roslaunch arti_can_interface can_interface.launch device_name:=vcan0 dump_file_name:=dump.txt command_file_name:=commands.txt command_interval:=2.0
```

A command file looks like this:

```
# Transmit CAN messages:
tx;0x185;1;2;3
tx;0;

# Fake reception of CAN messages:
rx;0x205;10;0x10;010
```

A dump file lists received and transmitted CAN messages in the same format.


## Sending CAN messages

The example below shows how to send a message with the CAN interface. There are several methods to send a message; the one shown is the simplest one.
See the documentation in [can_interface.h](arti_can_interface/include/arti_can_interface/can_interface.h) for further methods to send CAN messages.

```cpp
// Create the CAN interface for a specific device:
std::string device_name = "vcan0";
arti_can_interface::CanInterfacePtr can_interface = CanInterface::createCanInterface(device_name);

// Prepare the message data:
std::vector<uint8_t> data;
data.push_back(1);
data.push_back(2);
data.push_back(3);

// Use the specified id for the CAN message:
uint32_t id = 0x185;

// Send the message:
can_interface->sendMessage(id, data);
```

As an alternative one can also use the helper class `CanDataWriter`:
```cpp
// Create the CAN interface for a specific device:
std::string device_name = "vcan0";
arti_can_interface::CanInterfacePtr can_interface = CanInterface::createCanInterface(device_name);

// Prepare the message data:
arti_can_interface::CanDataWriter::Buffer data;
arti_can_interface::CanDataWriter writer{data};
writer.writeUnsignedInt8(1);
writer.writeUnsignedInt8(2);
writer.writeUnsignedInt8(3);

// Write more complicated data types
writer.writeUnsignedInt16(1234);
writer.writeFloatAsInt16(5678.9);

// Use the specified id for the CAN message:
uint32_t id = 0x185;

// Send the message:
can_interface->sendMessage(id, data);
```


## Receiving CAN messages

The example below shows how to register a callback for receiving messages with a specific id. The callback function will be called by the receiving thread, and will block other callback functions. Receiving data can be considered thread-save.

```cpp
// Create the CAN interface for a specific device:
std::string device_name = "vcan0";
arti_can_interface::CanInterfacePtr can_interface = CanInterface::createCanInterface(device_name);

// The CAN id we're interested in:
uint32_t id = 0x185;

// Subscribe to messages with the given id:
can_interface->subscribe(id, &canMessageCB);

...

// An example callback:
void canMessageCB(const arti_can_msgs::CanMessageConstPtr& can_message)
{
  ROS_INFO_STREAM("id: " << can_message->id);
  ROS_INFO_STREAM("extended_id: " << static_cast<int>(can_message->extended_id));
  ROS_INFO_STREAM("rtr: " << static_cast<int>(can_message->rtr));
  ROS_INFO_STREAM("extended_data_length: " << static_cast<int>(can_message->extended_data_length));
  ROS_INFO_STREAM("bit_rate_switch: " << static_cast<int>(can_message->bit_rate_switch));
  ROS_INFO_STREAM("error_state_indicator: " << static_cast<int>(can_message->error_state_indicator));

  for (const uint8_t data : can_message->data)
  {
    ROS_INFO_STREAM("new byte received: " << static_cast<int>(data));
  }
}
```

As an alternative one can also use the helper class `CanDataReader`:
```cpp
// Create the CAN interface for a specific device:
std::string device_name = "vcan0";
arti_can_interface::CanInterfacePtr can_interface = CanInterface::createCanInterface(device_name);

// The CAN id we're interested in:
uint32_t id = 0x185;

// Subscribe to messages with the given id:
can_interface->subscribe(id, &canMessageCB);

...

// An example callback:
void canMessageCB(const arti_can_msgs::CanMessageConstPtr& can_message)
{
  ROS_INFO_STREAM("id: " << can_message->id);
  ROS_INFO_STREAM("extended_id: " << static_cast<int>(can_message->extended_id));
  ROS_INFO_STREAM("rtr: " << static_cast<int>(can_message->rtr));
  ROS_INFO_STREAM("extended_data_length: " << static_cast<int>(can_message->extended_data_length));
  ROS_INFO_STREAM("bit_rate_switch: " << static_cast<int>(can_message->bit_rate_switch));
  ROS_INFO_STREAM("error_state_indicator: " << static_cast<int>(can_message->error_state_indicator));

  for (const uint8_t data : can_message->data)
  {
    ROS_INFO_STREAM("new byte received: " << static_cast<int>(data));
  }

  arti_can_interface::CanDataReader payload_reader(can_message->data.begin(), can_message->data.end());

  auto first_byte = payload_reader.readUnsignedInt8();
  auto second_byte = payload_reader.readUnsignedInt8();
  auto third_byte = payload_reader.readUnsignedInt8();

  // Read more complicated data types
  auto fourth_and_fifth_byte = payload_reader.readUnsignedInt16();
  auto sixth_and_seventh_byte = payload_reader.readInt16AsFloat();
}
```

## License

All files in this repository are distributed under the terms of the 2-clause BSD license. See [LICENSE.txt](LICENSE.txt) for details.
