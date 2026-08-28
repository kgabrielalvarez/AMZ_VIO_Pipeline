#include "orchestrator/orchestrator.hpp"

// Constructor
orchestrator::orchestrator() : Node("orchestrator") {

    // Declare parameters
    this->declare_parameter("camera_calibration_samples", 0);

    // Initialize publishers and susbcribers
    camera_calibration_samples_publisher_ = this->create_publisher<std_msgs::msg::Int32>("camera_calibration_samples",
        PUB_SUB_BUFFER_SIZE);
    camera_calibration_finished_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("camera_calibration_finished",
        PUB_SUB_BUFFER_SIZE, std::bind(&orchestrator::camera_calibration_finished_callback, this, std::placeholders::_1));
    right_image_subscriber_ = this->create_subscription<amz_vio_pipeline_msgs::msg::ImageIndexed>("right_images",
        PUB_SUB_BUFFER_SIZE, std::bind(&orchestrator::right_images_callback, this, std::placeholders::_1));
    left_image_subscriber_ = this->create_subscription<amz_vio_pipeline_msgs::msg::ImageIndexed>("left_images",
        PUB_SUB_BUFFER_SIZE, std::bind(&orchestrator::left_images_callback, this, std::placeholders::_1));
    camera_timestamp_subscriber_ = this->create_subscription<amz_vio_pipeline_msgs::msg::CameraTimestamps>("camera_timestamps",
        PUB_SUB_BUFFER_SIZE, std::bind(&orchestrator::camera_timestamp_callback, this, std::placeholders::_1));
    state_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("triggering_board_state", PUB_SUB_BUFFER_SIZE);
    state_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>("triggering_board_state", 
        PUB_SUB_BUFFER_SIZE, std::bind(&orchestrator::transition_handler_callback, this, std::placeholders::_1));

    // Initialize timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_PERIOD), std::bind(&orchestrator::timer_callback, this));

    // Wait for can_driver and camera_driver nodes to initialize
    rclcpp::sleep_for(std::chrono::milliseconds(NODE_INIT_WAIT));
    
    // Start pipeline
    transition_msg_.data = static_cast<uint8_t>(triggering_board_state::CAL_IMU);
    state_publisher_->publish(transition_msg_);

    // Publish number of images to take during CAL_CAM phase
    camera_calibration_samples_ = this->get_parameter("camera_calibration_samples").as_int();
    std::cout << "camera calibration samples = " << camera_calibration_samples_ << std::endl;
    camera_calibration_samples_msg_.data = camera_calibration_samples_;
    camera_calibration_samples_publisher_->publish(camera_calibration_samples_msg_);

}

// Destructor
orchestrator::~orchestrator() {

    transition_to_STOP();

}

void orchestrator::transition_to_STOP() {

    // Check that state transition is valid
    if ((triggering_board_state_ != triggering_board_state::CAL_IMU) &&
        (triggering_board_state_ != triggering_board_state::CAL_CAM) &&
        (triggering_board_state_ != triggering_board_state::RUN)) {
            throw std::runtime_error("Transitioning to STOP from unspecified state is invalid");
    }
    // Perform state transition
    triggering_board_state_ = triggering_board_state::STOP;

    // Do nothing in this state transition

    // Notify
    RCLCPP_INFO(this->get_logger(), "orchestrator node has entered %s", state_to_string(triggering_board_state_).c_str());

}

void orchestrator::transition_to_CAL_IMU() {

    // Check that state transition is valid
    // 1. Check that previous state was STOP
    if (triggering_board_state_ != triggering_board_state::STOP) {
        throw std::runtime_error("Transitioning to CAL_IMU from " + 
            state_to_string(triggering_board_state_) + " is invalid");
    }
    // Perform state transition
    triggering_board_state_ = triggering_board_state::CAL_IMU;

    // Do nothing in this state transition

    // Notify
    RCLCPP_INFO(this->get_logger(), "orchestrator node has entered %s", state_to_string(triggering_board_state_).c_str());

}

void orchestrator::transition_to_CAL_CAM() {

    // Check that state transition is valid
    // 1. Check that previous state was CAL_IMU
    if (triggering_board_state_ != triggering_board_state::CAL_IMU) {
        throw std::runtime_error("Transitioning to CAL_CAM from " + 
            state_to_string(triggering_board_state_) + " is invalid");
    }
    // Perform state transition
    triggering_board_state_ = triggering_board_state::CAL_CAM;

    // Do nothing in this state transition

    // Notify
    RCLCPP_INFO(this->get_logger(), "orchestrator node has entered %s", state_to_string(triggering_board_state_).c_str());

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

void orchestrator::camera_calibration_finished_callback(const std_msgs::msg::Bool::SharedPtr msg) {

    // Check that we received a "true" flag
    if (msg->data == false) {
        throw std::runtime_error("Received camera_calibration_finished = false");
    }

    // Set flag to "true"
    camera_calibration_finished_ = msg->data;

}

void orchestrator::left_images_callback(const amz_vio_pipeline_msgs::msg::ImageIndexed::SharedPtr msg) {

    switch (triggering_board_state_) {

        case triggering_board_state::CAL_CAM:

            // Update index
            left_image_index_++;

            // If this is the first message confirm that the index is 1
            if (left_image_index_ == 1) {
                if (msg->index != 1) {
                    throw std::runtime_error("First left image message does not have an index of 1");
                }
            }

            // If this is the last calibration message confirm that it's index is camera_calibration_samples_
            if (left_image_index_ == camera_calibration_samples_) {
                if (msg->index != camera_calibration_samples_) {
                    throw std::runtime_error("Last left image messsage does not have an index of " 
                        + std::to_string(camera_calibration_samples_));
                }
            }

            break;

        case triggering_board_state::RUN:

            // TO-DO

            break;

        default:

            throw std::runtime_error("Received left image message while in " + 
                state_to_string(triggering_board_state_) + " state");

    }

}

void orchestrator::right_images_callback(const amz_vio_pipeline_msgs::msg::ImageIndexed::SharedPtr msg) {

    switch (triggering_board_state_) {

        case triggering_board_state::CAL_CAM:

            // Update index
            right_image_index_++;

            // If this is the first message confirm that the index is 1
            if (right_image_index_ == 1) {
                if (msg->index != 1) {
                    throw std::runtime_error("First right image message does not have an index of 1");
                }
            }

            // If this is the last calibration message confirm that it's index is camera_calibration_samples_
            if (right_image_index_ == camera_calibration_samples_) {
                if (msg->index != camera_calibration_samples_) {
                    throw std::runtime_error("Last right image messsage does not have an index of " 
                        + std::to_string(camera_calibration_samples_));
                }
            }

            break;

        case triggering_board_state::RUN:

            // TO-DO

            break;

        default:

            throw std::runtime_error("Received left image message while in " + 
                state_to_string(triggering_board_state_) + " state");

    }
    
}

void orchestrator::camera_timestamp_callback(const amz_vio_pipeline_msgs::msg::CameraTimestamps::SharedPtr msg) {

    switch (triggering_board_state_) {

        case triggering_board_state::CAL_CAM:
            
            // Update index
            camera_timestamp_index_++;
            
            // If this is the first message confirm that it's index is 1
            if (camera_timestamp_index_ == 1) {
                if (msg->index != 1) {
                    throw std::runtime_error("First camera calibration timestamp message does not have an index of 1");
                }
            }

            // If this is the last calibration message confirm that it's index is camera_calibration_samples_
            if (camera_timestamp_index_ == camera_calibration_samples_) {
                if (msg->index != camera_calibration_samples_) {
                    throw std::runtime_error("Last camera calibration timestamp message does not have an index of " 
                        + std::to_string(camera_calibration_samples_));
                }
            }

            break;

        case triggering_board_state::RUN:
            
            // TO-DO

            break;

        default:

            throw std::runtime_error("Received camera timestamp message while in " + 
                state_to_string(triggering_board_state_) + " state");

    }

}

void orchestrator::timer_callback() {

    switch (triggering_board_state_) {

        case triggering_board_state::STOP:

            // Fall through

        case triggering_board_state::CAL_IMU:

            break;

        case triggering_board_state::CAL_CAM:

            // If possible, request transition to RUN
            if ((camera_calibration_finished_ == true) &&
                (camera_timestamp_index_ == camera_calibration_samples_) &&
                (left_image_index_ == camera_calibration_samples_) &&
                (right_image_index_ == camera_calibration_samples_)) {

                    transition_msg_.data = static_cast<uint8_t>(triggering_board_state::RUN);
                    state_publisher_->publish(transition_msg_);

            }

            break;

        case triggering_board_state::RUN:

            // TO-DO

            break;

        default:

            throw std::runtime_error("Timer callback was triggered from unknown state");

    }

}

// Main: code entry point
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<orchestrator>());
    rclcpp::shutdown();
    return 0;
}