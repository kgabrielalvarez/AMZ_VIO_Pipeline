// Code based on:
// 1. https://github.com/GOFIRST-Robotics/ros2socketcan_bridge/tree/master
// 2. https://docs.kernel.org/networking/can.html

#ifndef can_driver_h
#define can_driver_h

// Include CAN specific Libraries
#include "linux/can/raw.h"
#include <sys/ioctl.h>
#include <net/if.h>
#include <thread>

// Include ROS2 libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "amz_vio_pipeline_msgs/msg/calibration_timestamps.hpp"
#include "orchestrator/orchestrator_utils.hpp"

// Macros
// CAN message IDs
#define STATE_CAN_ID            0x001 // Highest priority
#define TIMESTAMPS_CAN_ID       0x002
#define FINISHED_CAN_ID         0x003
#define CAM_CAN_ID              0x004
#define IMU_CAN_ID              0x005 // Lowest priority

// State CAN message frame lengths
#define STOP_FRAME_LENGTH           1 // state (1 byte)
#define CAL_IMU_FRAME_LENGTH        9 // state (1 byte) + imu_calibration_duration_ (4 bytes) + imu_rate_ (4 bytes)
#define CAL_CAM_FRAME_LENGTH        5 // state (1 byte) + camera_calibration_rate_ (4 bytes)
#define RUN_FRAME_LENGTH            5 // state (1 byte) + camera_rate_ (4 bytes)

// CAN message specifying that CAL_IMU phase is complete
#define FINISHED_CAN_MSG         0xFF

// Delay between state transitions (use to give triggering board time to transition between states)
#define STATE_SWITCH_DELAY       1000 // [ms]

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
        struct timeval timeout_;
        int enable_can_fd_ = 1;
        struct sockaddr_can can_socket_address_;

        // read_can() thread
        std::thread can_reader_thread_;
        struct canfd_frame frame_;
        ssize_t num_bytes_read_;

        // Variables to publish in IMU msg
        bytes_to_float_t acceleration_x_;     // [m/s^2]
        bytes_to_float_t acceleration_y_;     // [m/s^2]
        bytes_to_float_t acceleration_z_;     // [m/s^2]
        bytes_to_float_t angular_velocity_x_; // [rad/s]
        bytes_to_float_t angular_velocity_y_; // [rad/s]
        bytes_to_float_t angular_velocity_z_; // [rad/s]
        bytes_to_float_t timestamp_;          // [ms]

        // Variables to publish in calibration_timestamps msg
        uint32_t imu_timestamp_;
        uint32_t mcu_timestamp_;

        // Variables to publish in camera_timestamp msg
        uint32_t cam_timestamp_;

        // Unit conversion factors
        float mg_to_ms2_ = 9.81/1000.0;
        float dps_to_rps_ = 3.1415926535/180.0;

        // Messages to publish
        amz_vio_pipeline_msgs::msg::CalibrationTimestamps calibration_timestamps_msg_;
        builtin_interfaces::msg::Time cam_msg_;
        sensor_msgs::msg::Imu imu_msg_;

        // Write to CAN Bus
        ssize_t num_bytes_write_;
        struct canfd_frame stop_frame_;
        struct canfd_frame cal_imu_frame_;
        struct canfd_frame cal_cam_frame_;
        struct canfd_frame run_frame_;

        // Camera and IMU rates
        int32_t camera_rate_; // [FPS] User defined in YAML file
        int32_t camera_calibration_rate_ = 10; // [FPS]
        int32_t imu_rate_; // [Hz] User defined in YAML file
        int32_t imu_calibration_timestamps_ = 5*208; // [s]

        // Camera and IMU rate bounds
        int32_t camera_rate_max_ = 168; // [FPS] max frame rate that the camera can achieve: https://www.baslerweb.com/en/shop/a2a1920-168mgc/
        int32_t camera_rate_min_ = 5; // [FPS] TO-DO: think about this more deeply, I just made this up 

        // Counter to keep track of the number of calibration timestamps that have been received
        int32_t calibration_timestamp_counter_ = 0;

        // Triggering board state and requested state to transition to
        triggering_board_state triggering_board_state_;
        triggering_board_state state_to_transition_to_;

        // Publishers and subscriber
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_and_timestamp_publisher_;
        rclcpp::Publisher<amz_vio_pipeline_msgs::msg::CalibrationTimestamps>::SharedPtr calibration_timestamps_publisher_;
        rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr cam_timestamp_publisher_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_subscriber_;

        // Methods: state transitions
        void transition_to_STOP();
        void transition_to_CAL_IMU();
        void transition_to_CAL_CAM();
        void transition_to_RUN();

        // Methods: miscellaneous
        void transition_handler_callback(const std_msgs::msg::UInt8::SharedPtr msg);
        void configure_can_socket();
        void read_can();
        void read_state_can_msg();
        void read_finished_can_msg();
        void read_timestamps_can_msg();
        void read_cam_can_msg();
        void read_imu_can_msg();

};

#endif