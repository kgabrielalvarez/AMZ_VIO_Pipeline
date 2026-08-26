#include "orchestrator/orchestrator.hpp"

// Constructor
orchestrator::orchestrator() : Node("orchestrator") {

    // Initialize publishers and susbcribers
    state_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("triggering_board_state", 10);
    state_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>("triggering_board_state", 
        10, std::bind(&orchestrator::transition_handler_callback, this, std::placeholders::_1));

    // Wait for can_driver and camera_driver nodes to initialize
    rclcpp::sleep_for(std::chrono::milliseconds(NODE_INIT_WAIT));
    
    // Start pipeline
    transition_msg_.data = static_cast<uint8_t>(triggering_board_state::CAL_IMU);
    state_publisher_->publish(transition_msg_);

}

// Destructor
orchestrator::~orchestrator() {

    transition_to_STOP();

}

void orchestrator::transition_to_STOP() {

}

void orchestrator::transition_to_CAL_IMU() {

}

void orchestrator::transition_to_CAL_CAM() {

}

void orchestrator::transition_to_RUN() {

}

void orchestrator::transition_handler_callback(const std_msgs::msg::UInt8::SharedPtr msg) {

    // Update requested state to transition to
    state_to_transition_to_ = static_cast<triggering_board_state>(msg->data);

    // Transition to commanded state
    switch(state_to_transition_to_) {

        case triggering_board_state::STOP:
            transition_to_STOP();
            break;

        case triggering_board_state::CAL_IMU:
            transition_to_CAL_IMU();
            break;

        case triggering_board_state::CAL_CAM:
            transition_to_CAL_CAM();
            break;

        case triggering_board_state::RUN:
            transition_to_RUN();
            break;

        default:
            throw std::runtime_error("Requested state transition is not valid");

    }

}

// Main: code entry point
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<orchestrator>());
    rclcpp::shutdown();
    return 0;
}