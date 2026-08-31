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
    left_synchronized_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("left_synchronized_images", PUB_SUB_BUFFER_SIZE);
    right_synchronized_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("right_synchronized_images", PUB_SUB_BUFFER_SIZE);
    state_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("triggering_board_state", PUB_SUB_BUFFER_SIZE);
    state_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>("triggering_board_state", 
        PUB_SUB_BUFFER_SIZE, std::bind(&orchestrator::transition_handler_callback, this, std::placeholders::_1));

    // Initialize timers but don't start running them immediately
    timer_CAL_CAM_ = this->create_wall_timer(std::chrono::milliseconds(CAL_CAM_TIMER_PERIOD), std::bind(&orchestrator::timer_CAL_CAM_callback, this));
    timer_CAL_CAM_->cancel();
    timer_RUN_ = this->create_wall_timer(std::chrono::milliseconds(RUN_TIMER_PERIOD), std::bind(&orchestrator::timer_RUN_callback, this));
    timer_RUN_->cancel();

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

    // Check that the state transition is valid
    // 1. Check that the previous state was not STOP
    if ((triggering_board_state_ != triggering_board_state::CAL_IMU) &&
        (triggering_board_state_ != triggering_board_state::CAL_CAM) &&
        (triggering_board_state_ != triggering_board_state::RUN)) {
            throw std::runtime_error("Transitioning to STOP from unspecified state is invalid");
    }
    // Perform state transition
    triggering_board_state_ = triggering_board_state::STOP;

    // Stop all timers
    timer_CAL_CAM_->cancel();
    timer_RUN_->cancel();

    // Notify
    RCLCPP_INFO(this->get_logger(), "orchestrator node has entered %s", state_to_string(triggering_board_state_).c_str());

}

void orchestrator::transition_to_CAL_IMU() {

    // Check that the state transition is valid
    // 1. Check that the previous state was STOP
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

    // Check that the state transition is valid
    // 1. Check that the previous state was CAL_IMU
    if (triggering_board_state_ != triggering_board_state::CAL_IMU) {
        throw std::runtime_error("Transitioning to CAL_CAM from " + 
            state_to_string(triggering_board_state_) + " is invalid");
    }
    // Perform state transition
    triggering_board_state_ = triggering_board_state::CAL_CAM;

    // Start CAL_CAM timer
    timer_CAL_CAM_->reset();

    // Notify
    RCLCPP_INFO(this->get_logger(), "orchestrator node has entered %s", state_to_string(triggering_board_state_).c_str());

}

void orchestrator::transition_to_RUN() {

    // Check that the state transition is valid
    // 1. Check that the previous state was CAL_CAM
    if (triggering_board_state_ != triggering_board_state::CAL_CAM) {
        throw std::runtime_error("Transitioning to RUN from " + 
            state_to_string(triggering_board_state_) + " is invalid");
    }
    // Perform state transition
    triggering_board_state_ = triggering_board_state::RUN;

    // Stop CAL_CAM timer and start RUN timer
    timer_CAL_CAM_->cancel();
    timer_RUN_->reset();

    // Notify
    RCLCPP_INFO(this->get_logger(), "orchestrator node has entered %s", state_to_string(triggering_board_state_).c_str());

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

            // Update index
            left_image_index_++;

            // Check that message index matches expected index
            if (msg->index != left_image_index_) {
                throw std::runtime_error(std::string("Expected an index of ") + std::to_string(left_image_index_) + 
                    std::string(" on left_images topic but received an index of " + std::to_string(msg->index)));
            }

            // Add image to buffer
            left_image_.index = msg->index;
            left_image_.image = msg->image;
            left_image_buffer_.push_back(left_image_);

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

            // Update index
            right_image_index_++;

            // Check that message index matches expected index
            if (msg->index != right_image_index_) {
                throw std::runtime_error(std::string("Expected an index of ") + std::to_string(right_image_index_) + 
                    std::string(" on right_images topic but received an index of " + std::to_string(msg->index)));
            }

            // Add images to buffer
            right_image_.index = msg->index;
            right_image_.image = msg->image;
            right_image_buffer_.push_back(right_image_);

            break;

        default:

            throw std::runtime_error("Received right image message while in " + 
                state_to_string(triggering_board_state_) + " state");

    }
    
}

void orchestrator::camera_timestamp_callback(const amz_vio_pipeline_msgs::msg::CameraTimestamps::SharedPtr msg) {

    switch (triggering_board_state_) {

        case triggering_board_state::CAL_CAM:

            switch(msg->camera_index) {
                
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
            
            switch(msg->camera_index) {
                
                case (LEFT_CAM_ID):
                    
                    // Update index
                    left_camera_timestamp_index_++;

                    // Check that message index matches expected index
                    if (msg->frame_index != left_camera_timestamp_index_) {
                        throw std::runtime_error(std::string("Expected an index of ") + std::to_string(left_camera_timestamp_index_) + 
                            std::string(" on camera_timestamps (left) topic but received an index of ") + std::to_string(msg->frame_index));
                    }

                    // Add timestamps to buffer
                    left_timestamp_.index = msg->frame_index;
                    left_timestamp_.timestamp = msg->timestamp;
                    left_timestamp_buffer_.push_back(left_timestamp_);

                    break;

                case (RIGHT_CAM_ID):

                    // Update index
                    right_camera_timestamp_index_++;

                    // Check that message index matches expected index
                    if (msg->frame_index != right_camera_timestamp_index_) {
                        throw std::runtime_error(std::string("Expected an index of ") + std::to_string(right_camera_timestamp_index_) + 
                            std::string(" on camera_timestamps (right) topic but received an index of ") + std::to_string(msg->frame_index));
                    }

                    // Add timestamps to buffer
                    right_timestamp_.index = msg->frame_index;
                    right_timestamp_.timestamp = msg->timestamp;
                    right_timestamp_buffer_.push_back(right_timestamp_);

                    break;

                default:

                    throw std::runtime_error("Unkown camera index in camera timestamp message");

            }

            break;

        default:

            throw std::runtime_error("Received camera timestamp message while in " + 
                state_to_string(triggering_board_state_) + " state");

    }

}

void orchestrator::timer_CAL_CAM_callback() {

    // Check that the timer_CAL_CAM_callback is being run from the CAL_CAM state
    if (triggering_board_state_ != triggering_board_state::CAL_CAM) {
        throw std::runtime_error("timer_CAL_CAM_callback was run from " + 
            state_to_string(triggering_board_state_) + ", which is invalid");
    }

    // If possible, request transition to RUN
    if (camera_calibration_finished_ == true) {
        if ((left_image_index_ == total_left_calibration_samples_) &&
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

                // Calculate frame offset
                if (total_left_calibration_samples_ == total_right_calibration_samples_) {
                    offset_ = 0;
                }
                else if (total_left_calibration_samples_ < total_right_calibration_samples_) {
                    offset_ = total_left_calibration_samples_ - total_left_calibration_samples_;
                }
                else {
                    throw std::runtime_error(std::string("total_left_calibration_samples_ >"
                        "total_right_calibration_samples_, not valid: ") + 
                        std::to_string(total_left_calibration_samples_) + std::string(" > ") + 
                        std::to_string(total_right_calibration_samples_));
                }

                // Notify
                RCLCPP_INFO(this->get_logger(), "Offset = %d", offset_);

                transition_msg_.data = static_cast<uint8_t>(triggering_board_state::RUN);
                state_publisher_->publish(transition_msg_);

        }
    }

}

void orchestrator::timer_RUN_callback() {

    // Check that the timer_RUN_callback is being run from the RUN state
    if (triggering_board_state_ != triggering_board_state::RUN) {
        throw std::runtime_error("timer_RUN_callback was run from " +
            state_to_string(triggering_board_state_) + " , which is invalid");
    }

    // Check that buffers aren't empty
    if ((!left_image_buffer_.empty()) &&
        (!right_image_buffer_.empty()) &&
        (!left_timestamp_buffer_.empty()) &&
        (!right_timestamp_buffer_.empty())) {

        // Get buffer elements
        left_image_retrieved_ = left_image_buffer_.front();
        left_image_buffer_.pop_front();
        right_image_retrieved_ = right_image_buffer_.front();
        right_image_buffer_.pop_front();
        left_timestamp_retrieved_ = left_timestamp_buffer_.front();
        left_timestamp_buffer_.pop_front();
        right_timestamp_retrieved_ = right_timestamp_buffer_.front();
        right_timestamp_buffer_.pop_front();
        
        // Confirm that left indices match
        if (left_image_retrieved_.index != left_timestamp_retrieved_.index) {
            throw std::runtime_error(std::string("Left image index ") + std::to_string(left_image_retrieved_.index) +
                std::string(" does not match left timestamp index ") + std::to_string(left_timestamp_retrieved_.index));
        }

        // Confirm that right indices match
        if (right_image_retrieved_.index != right_timestamp_retrieved_.index) {
            throw std::runtime_error(std::string("Right image index ") + std::to_string(right_image_retrieved_.index) +
                std::string(" does not match right timestamp index ") + std::to_string(right_timestamp_retrieved_.index));
        }

        // Confirm that left and right indices match
        if (left_image_retrieved_.index != (right_image_retrieved_.index + offset_)) {
            throw std::runtime_error(std::string("Left image index ") + std::to_string(left_image_retrieved_.index) +
                std::string(" does not match right image index ") + std::to_string(right_image_retrieved_.index + offset_));
        }

        // Confirm that left and right timestamps match
        timestamp_error_ns_ = static_cast<int64_t>(std::abs(rclcpp::Time(left_timestamp_retrieved_.timestamp).nanoseconds() - 
            rclcpp::Time(right_timestamp_retrieved_.timestamp).nanoseconds()));
        if (timestamp_error_ns_ > EPSILON) {
            throw std::runtime_error(std::string("Left and right timestamp have an error of = ") + 
                std::to_string(timestamp_error_ns_) + " ns");
        }

        // Publish images with left timestamp
        left_synchronized_image_ = left_image_retrieved_.image;
        left_synchronized_image_.header.stamp = left_timestamp_retrieved_.timestamp;
        right_synchronized_image_ = right_image_retrieved_.image;
        right_synchronized_image_.header.stamp = left_timestamp_retrieved_.timestamp;
        left_synchronized_image_publisher_->publish(left_synchronized_image_);
        right_synchronized_image_publisher_->publish(right_synchronized_image_);

    }
}

// Main: code entry point
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<orchestrator>());
    rclcpp::shutdown();
    return 0;
}