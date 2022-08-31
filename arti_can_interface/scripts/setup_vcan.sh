#!/bin/sh

# This creates a virtual CAN interface with the given name.

sudo modprobe vcan
sudo ip link add dev "$1" type vcan
sudo ip link set up "$1"
ip link show "$1"

