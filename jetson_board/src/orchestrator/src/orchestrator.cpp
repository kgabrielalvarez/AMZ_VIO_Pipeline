#include "orchestrator/orchestrator.hpp"

// Constructor
orchestrator::orchestrator() : Node("orchestrator") {

    // Declare parameters
    this->declare_parameter("camera_calibration_samples", 0);

    // Set camera_calibration_finished_ flag to false
    camera_calibration_finished_ = false;

    // Initialize publishers and susbcribers
    camera_calibration_samples_publisher_ = this->create_publisher<std_msgs::msg::Int32>("camera_calibration_samples",
        PUB_SUB_BUFFER_SIZE);
    camera_calibration_finished_subscriber_ = this->create_subscription<amz_vio_pipeline_msgs::msg::CameraCalibrationFinished>
        ("camera_calibration_finished", PUB_SUB_BUFFER_SIZE, std::bind(&orchestrator::camera_calibration_finished_callback, 
         this, std::placeholders::_1));
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

void orchestrator::camera_calibration_finished_callback(const amz_vio_pipeline_msgs::msg::CameraCalibrationFinished::SharedPtr msg) {

    // Check that we received a "true" flag
    if (msg->finished == false) {
        throw std::runtime_error("Received camera_calibration_finished = false");
    }

    // Set flag to "true" and total number of calibration samples
    camera_calibration_finished_ = msg->finished;
    total_left_calibration_samples_ = msg->total_left_calibration_samples;
    total_right_calibration_samples_ = msg->total_right_calibration_samples;

    // Check that the total number of right calibration samples is what we expected
    if (total_right_calibration_samples_ != camera_calibration_samples_) {
        throw std::runtime_error(std::string("Expected a total of ") + std::to_string(camera_calibration_samples_) + 
            std::string(" right calibration samples, but the STM32 only logged ") + std::to_string(total_right_calibration_samples_));
    }

    // Notify
    RCLCPP_INFO(this->get_logger(), "Triggering board has finished camera calibration");

}

void orchestrator::left_images_callback(const amz_vio_pipeline_msgs::msg::ImageIndexed::SharedPtr msg) {

    switch (triggering_board_state_) {

        case triggering_board_state::CAL_CAM:

            // Update index
            left_image_index_++;

            // Check that message index matches expected index
            if (msg->index != left_image_index_) {
                throw std::runtime_error(std::string("Expected an index of ") + std::to_string(left_image_index_) + 
                    std::string(" on left_images topic but received an index of " + std::to_string(msg->index)));
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

            // Check that message index matches expected index
            if (msg->index != right_image_index_) {
                throw std::runtime_error(std::string("Expected an index of ") + std::to_string(right_image_index_) + 
                    std::string(" on right_images topic but received an index of " + std::to_string(msg->index)));
            }

            break;

        case triggering_board_state::RUN:

            // TO-DO

            break;

        default:

            throw std::runtime_error("Received right image message while in " + 
                state_to_string(triggering_board_state_) + " state");

    }
    
}

void orchestrator::camera_timestamp_callback(const amz_vio_pipeline_msgs::msg::CameraTimestamps::SharedPtr msg) {

    switch (triggering_board_state_) {

        case triggering_board_state::CAL_CAM:

            switch(msg->frame_index) {
                
                case (LEFT_CAM_ID):
                    
                    // Update index
                    left_camera_timestamp_index_++;

                    // Check that message index matches expected index
                    if (msg->frame_index != left_camera_timestamp_index_) {
                        throw std::runtime_error(std::string("Expected an index of ") + std::to_string(left_camera_timestamp_index_) + 
                            std::string(" on camera_timestamps (left) topic but received an index of ") + std::to_string(msg->frame_index));
                    }

                    break;

                case (RIGHT_CAM_ID):

                    // Update index
                    right_camera_timestamp_index_++;

                    // Check that message index matches expected index
                    if (msg->frame_index != right_camera_timestamp_index_) {
                        throw std::runtime_error(std::string("Expected an index of ") + std::to_string(right_camera_timestamp_index_) + 
                            std::string(" on camera_timestamps (right) topic but received an index of ") + std::to_string(msg->frame_index));
                    }

                    break;

                default:

                    throw std::runtime_error("Unkown camera index in camera timestamp message");

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
                (left_image_index_ == total_left_calibration_samples_) &&
                (right_image_index_ == total_right_calibration_samples_) &&
                (left_camera_timestamp_index_ == total_left_calibration_samples_) &&
                (right_camera_timestamp_index_ == total_right_calibration_samples_)) {

                    // Notify
                    RCLCPP_INFO(this->get_logger(), "Camera calibration was successfull!");
                    RCLCPP_INFO(this->get_logger(), "Received %d/%d left camera images", 
                        left_image_index_, total_left_calibration_samples_);
                    RCLCPP_INFO(this->get_logger(), "Received %d/%d right camera images", 
                        right_image_index_, total_right_calibration_samples_);
                    RCLCPP_INFO(this->get_logger(), "Received %d/%d left camera timestamps", 
                        left_camera_timestamp_index_, total_left_calibration_samples_);
                    RCLCPP_INFO(this->get_logger(), "Received %d/%d right camera timestamps", 
                        right_camera_timestamp_index_, total_right_calibration_samples_);

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