#!/bin/bash

# Cleanup function
cleanup() {
    sudo ip link set can0 down
}

trap cleanup EXIT INT

# Source
source install/setup.bash

# Configure CAN FD
sudo ip link set can0 type can bitrate 500000 dbitrate 500000 berr-reporting on fd on
sudo ip link set can0 up

# Start Nodes
ros2 launch bringup AMZ_VIO_Pipeline.launch.py
