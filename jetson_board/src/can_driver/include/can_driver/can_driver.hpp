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

// Type for converting uint8_t to float
union bytes_to_float_t {
    uint8_t as_bytes[4];
    float as_float;
    uint32_t as_uint32;
};

// Driver to connect CAN to ROS2
class can_driver : public rclcpp::Node {

    public: 

        // Constructor
        can_driver();

        // Destructor
        ~can_driver();

    private:

        // Variables
        // See: https://docs.kernel.org/networking/can.html
        // ("How to use SocketCAN" section)

        // CAN setup
        std::string can_socket_name_ = "can0";
        struct ifreq interface_;
        int socket_;
        int enable_can_fd_ = 1;
        struct sockaddr_can can_socket_address_;

        // read_can() thread
        std::thread can_reader_thread_;
        std::atomic<bool> run_triggering_board_;
        struct canfd_frame frame_;
        ssize_t num_bytes_read_;

        // Variables to publish
        bytes_to_float_t acceleration_x_;     // [m/s^2]
        bytes_to_float_t acceleration_y_;     // [m/s^2]
        bytes_to_float_t acceleration_z_;     // [m/s^2]
        bytes_to_float_t angular_velocity_x_; // [rad/s]
        bytes_to_float_t angular_velocity_y_; // [rad/s]
        bytes_to_float_t angular_velocity_z_; // [rad/s]
        bytes_to_float_t timestamp_;          // [ms]

        // Unit conversion factors
        float mg_to_ms2_ = 9.81/1000.0;
        float dps_to_rps_ = 3.1415926535/180.0;

        // Message to publish
        sensor_msgs::msg::Imu imu_message_;

        // Write to CAN Bus
        ssize_t num_bytes_write_;
        struct canfd_frame start_triggering_board_frame_;
        struct canfd_frame stop_triggering_board_frame_;

        // Publisher & Subscriber
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;

        // Functions
        void read_can();
        void start_triggering_board();
        void stop_triggering_board();
        void subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg);

};

#endif