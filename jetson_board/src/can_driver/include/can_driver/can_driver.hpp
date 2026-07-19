// Code based on:
// 1. https://github.com/GOFIRST-Robotics/ros2socketcan_bridge/tree/master
// 2. https://docs.kernel.org/networking/can.html

#ifndef can_driver_h
#define can_driver_h

// Include CAN specific Libraries
#include "linux/can/raw.h"
#include <sys/ioctl.h>
#include <net/if.h>

// Include ROS2 libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Driver to connect CAN to ROS2
class can_driver : public rclcpp::Node {

    public: 

        // Constructor
        can_driver();

    private:

        // Variables
        // See: https://docs.kernel.org/networking/can.html
        // ("How to use SocketCAN" section)

        // CAN setup
        std::string can_socket_name_ = "can0";
        struct ifreq interface_;
        int socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        struct sockaddr_can can_socket_address_;

        // read_can() thread
        std::thread can_reader_thread_;
        std::atomic<bool> run_triggering_board_;
        struct can_frame frame_;
        ssize_t num_bytes_read_;

        // Publisher & Subscriber
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;

        // Functions
        void start_can();
        void read_can();
        void stop_can();
        void start_triggering_board();
        void stop_triggering_board();
        void subscriber_callback(const std_msgs::msg::Bool msg);

};

#endif