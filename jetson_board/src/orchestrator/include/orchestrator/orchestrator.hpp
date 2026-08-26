#ifndef orchestrator_h
#define orchestrator_h

// Include ROS2 libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "orchestrator/orchestrator_utils.hpp"

// Include miscellaneous libraries
#include <chrono>

// Macros
// Delay between checks of number of nodes
#define CHECK_NUM_NODES_DELAY   100 // [ms]
// Number of nodes that should be listening to the Orchestrator
#define NUM_NODES                 2

// Node to orchestrate the pipeline
class orchestrator : public rclcpp::Node {

    public:

        // Constructor
        orchestrator();

        // Destructor
        ~orchestrator();

    private:

        // Triggering board state and requested state to transition to
        triggering_board_state triggering_board_state_;
        triggering_board_state state_to_transition_to_;

        // Transition command message
        std_msgs::msg::UInt8 transition_msg_;

        // Publishers and subscribers
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_subscriber_;

        // Methods: state transitions
        void transition_to_STOP();
        void transition_to_CAL_IMU();
        void transition_to_CAL_CAM();
        void transition_to_RUN();

        // Methods: miscellaneous
        void transition_handler_callback(const std_msgs::msg::UInt8::SharedPtr msg);
};

#endif