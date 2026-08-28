#ifndef orchestrator_h
#define orchestrator_h

// Include ROS2 libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "orchestrator/orchestrator_utils.hpp"
#include "amz_vio_pipeline_msgs/msg/camera_timestamps.hpp"
#include "amz_vio_pipeline_msgs/msg/image_indexed.hpp"

// Include miscellaneous libraries
#include <chrono>

// Macros
// Delay between checks of number of nodes
#define NODE_INIT_WAIT   500 // [ms]
// Number of nodes that should be listening to the Orchestrator
#define NUM_NODES          2
// Period for the timer callback
#define TIMER_PERIOD      10 // [,s]

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
        bool camera_calibration_finished_ = false;

        // Indexes to keep track of images and timestamps
        uint32_t left_image_index_ = 0;
        uint32_t right_image_index_ = 0;
        uint32_t camera_timestamp_index_ = 0;

        // Publishers and subscribers
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr camera_calibration_samples_publisher_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_calibration_finished_subscriber_;
        rclcpp::Subscription<amz_vio_pipeline_msgs::msg::ImageIndexed>::SharedPtr left_image_subscriber_;
        rclcpp::Subscription<amz_vio_pipeline_msgs::msg::ImageIndexed>::SharedPtr right_image_subscriber_;
        rclcpp::Subscription<amz_vio_pipeline_msgs::msg::CameraTimestamps>::SharedPtr camera_timestamp_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_subscriber_;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Callbacks
        void transition_handler_callback(const std_msgs::msg::UInt8::SharedPtr msg);
        void camera_calibration_finished_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void left_images_callback(const amz_vio_pipeline_msgs::msg::ImageIndexed::SharedPtr msg);
        void right_images_callback(const amz_vio_pipeline_msgs::msg::ImageIndexed::SharedPtr msg);
        void camera_timestamp_callback(const amz_vio_pipeline_msgs::msg::CameraTimestamps::SharedPtr msg);
        void timer_callback();

        // Methods: state transitions
        void transition_to_STOP();
        void transition_to_CAL_IMU();
        void transition_to_CAL_CAM();
        void transition_to_RUN();

};

#endif