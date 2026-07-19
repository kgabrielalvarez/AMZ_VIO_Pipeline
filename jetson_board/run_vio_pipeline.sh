#!/bin/bash

# Cleanup function
cleanup() {
    sudo ip link set can0 down
}

trap cleanup EXIT INT

# Source
source install/setup.bash

# Configure CAN
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Start Nodes
ros2 run can_driver can_driver