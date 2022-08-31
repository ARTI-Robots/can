#!/bin/sh

# This deletes the virtual gateways and virtual CAN interfaces created via setup_connected_vcans.sh.

sudo modprobe can-gw
sudo cangw -D -s "$2" -d "$1" -e
sudo cangw -D -s "$1" -d "$2" -e

rosrun arti_can_interface delete_vcan.sh "$2"
rosrun arti_can_interface delete_vcan.sh "$1"

