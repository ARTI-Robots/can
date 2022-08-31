#!/bin/sh

# This creates two virtual CAN interfaces with virtual gateways connecting their virtual networks.

rosrun arti_can_interface setup_vcan.sh "$1"
rosrun arti_can_interface setup_vcan.sh "$2"

sudo modprobe can-gw
sudo cangw -A -s "$1" -d "$2" -e
sudo cangw -A -s "$2" -d "$1" -e

