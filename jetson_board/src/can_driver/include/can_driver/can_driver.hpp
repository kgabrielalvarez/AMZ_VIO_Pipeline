// Code based on: https://github.com/GOFIRST-Robotics/ros2socketcan_bridge/tree/master

#ifndef can_driver_h
#define can_driver_h

// Include Libraries
#include <linux/can/raw.h>
#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"

// Driver to connect CAN to ROS2
class can_driver : public rclcpp::Node {

    // Public
    public: 

        // Constructor
        can_driver();

    // Private
    private:


}

#endif