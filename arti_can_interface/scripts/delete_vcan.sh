#!/bin/sh

# This deletes a virtual CAN interface with the given name.

sudo modprobe vcan
sudo ip link set down "$1"
sudo ip link delete dev "$1"

