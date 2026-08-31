#ifndef orchestrator_h
#define orchestrator_h

// Include ROS2 libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "orchestrator/orchestrator_utils.hpp"
#include "amz_vio_pipeline_msgs/msg/camera_timestamps.hpp"
#include "amz_vio_pipeline_msgs/msg/image_indexed.hpp"
#include "amz_vio_pipeline_msgs/msg/camera_calibration_finished.hpp"

// Include miscellaneous libraries
#include <chrono>
#include <deque>
#include <cmath>

// Macros
// Delay between checks of number of nodes
#define NODE_INIT_WAIT           500 // [ms]
// Number of nodes that should be listening to the Orchestrator
#define NUM_NODES                  2
// Period for the timer callback
#define CAL_CAM_TIMER_PERIOD      10 // [ms]
#define RUN_TIMER_PERIOD           5 // [ms] <-- should be less than or equal to the camera period
// Acceptable error between camera timestamps
#define EPSILON                50000 // [ns] 50 us

// Custom structs used for image synchronization
struct indexed_image {
    uint32_t index;
    sensor_msgs::msg::Image image;
};

struct indexed_timestamp {
    uint32_t index;
    builtin_interfaces::msg::Time timestamp;
};

// Node to orchestrate the pipeline
class orchestrator : public rclcpp::Node {

    public:

        // Constructor
        orchestrator();

        // Destructor
        ~orchestrator();

    private:

        // Number of images to take during camera calibration phase
        int32_t camera_calibration_samples_; // User defined in YAML file

        // Triggering board state and requested state to transition to
        triggering_board_state triggering_board_state_;
        triggering_board_state state_to_transition_to_;

        // Transition command message
        std_msgs::msg::UInt8 transition_msg_;

        // Number of camera calibration samples message
        std_msgs::msg::Int32 camera_calibration_samples_msg_;

        // Camera calibration finished flag
        bool camera_calibration_finished_;

        // Synchronized image messages
        sensor_msgs::msg::Image left_synchronized_image_;
        sensor_msgs::msg::Image right_synchronized_image_;

        // Indexes to keep track of images and timestamps
        uint32_t left_image_index_ = 0;
        uint32_t right_image_index_ = 0;
        uint32_t left_camera_timestamp_index_ = 0;
        uint32_t right_camera_timestamp_index_ = 0;

        // Total number of calibration samples
        uint32_t total_left_calibration_samples_;
        uint32_t total_right_calibration_samples_;

        // Queues to store images and timestamps
        std::deque<indexed_image> left_image_buffer_;
        std::deque<indexed_image> right_image_buffer_;
        std::deque<indexed_timestamp> left_timestamp_buffer_;
        std::deque<indexed_timestamp> right_timestamp_buffer_;

        // Elements to add to buffer
        indexed_image left_image_;
        indexed_image right_image_;
        indexed_timestamp left_timestamp_;
        indexed_timestamp right_timestamp_;

        // Elements to retrieve from buffer
        indexed_image left_image_retrieved_;
        indexed_image right_image_retrieved_;
        indexed_timestamp left_timestamp_retrieved_;
        indexed_timestamp right_timestamp_retrieved_;

        // Error between left and right timestamps
        int64_t timestamp_error_ns_; // [ns]

        // Offset between left and right image indices
        uint32_t offset_;

        // Publishers and subscribers
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr camera_calibration_samples_publisher_;
        rclcpp::Subscription<amz_vio_pipeline_msgs::msg::CameraCalibrationFinished>::SharedPtr camera_calibration_finished_subscriber_;
        rclcpp::Subscription<amz_vio_pipeline_msgs::msg::ImageIndexed>::SharedPtr left_image_subscriber_;
        rclcpp::Subscription<amz_vio_pipeline_msgs::msg::ImageIndexed>::SharedPtr right_image_subscriber_;
        rclcpp::Subscription<amz_vio_pipeline_msgs::msg::CameraTimestamps>::SharedPtr camera_timestamp_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_synchronized_image_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_synchronized_image_publisher_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_subscriber_;

        // Timers
        rclcpp::TimerBase::SharedPtr timer_CAL_CAM_;
        rclcpp::TimerBase::SharedPtr timer_RUN_;

        // Callbacks
        void transition_handler_callback(const std_msgs::msg::UInt8::SharedPtr msg);
        void camera_calibration_finished_callback(const amz_vio_pipeline_msgs::msg::CameraCalibrationFinished::SharedPtr msg);
        void left_images_callback(const amz_vio_pipeline_msgs::msg::ImageIndexed::SharedPtr msg);
        void right_images_callback(const amz_vio_pipeline_msgs::msg::ImageIndexed::SharedPtr msg);
        void camera_timestamp_callback(const amz_vio_pipeline_msgs::msg::CameraTimestamps::SharedPtr msg);
        void timer_CAL_CAM_callback();
        void timer_RUN_callback();

        // Methods: state transitions
        void transition_to_STOP();
        void transition_to_CAL_IMU();
        void transition_to_CAL_CAM();
        void transition_to_RUN();

};

#endif