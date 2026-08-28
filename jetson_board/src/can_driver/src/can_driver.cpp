#include "can_driver/can_driver.hpp"

// Constructor
can_driver::can_driver() : Node("can_driver") {

    // Declare parameters
    this->declare_parameter("camera_rate", 0); // [FPS]
    this->declare_parameter("imu_rate", 0); // [Hz]

    // Initialize publishers and subscriber
    imu_and_timestamp_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_and_timestamps", PUB_SUB_BUFFER_SIZE);
    calibration_timestamps_publisher_ = this->create_publisher<amz_vio_pipeline_msgs::msg::CalibrationTimestamps>
        ("calibration_timestamps", PUB_SUB_BUFFER_SIZE);
    camera_timestamp_publisher_ = this->create_publisher<amz_vio_pipeline_msgs::msg::CameraTimestamps>("camera_timestamps", PUB_SUB_BUFFER_SIZE);
    camera_calibration_samples_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("camera_calibration_samples",
        PUB_SUB_BUFFER_SIZE, std::bind(&can_driver::camera_calibration_samples_callback, this, std::placeholders::_1));
    camera_calibration_finished_publisher_ = this->create_publisher<std_msgs::msg::Bool>("camera_calibration_finished", 
        PUB_SUB_BUFFER_SIZE);
    state_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("triggering_board_state", PUB_SUB_BUFFER_SIZE);
    state_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>("triggering_board_state", 
        PUB_SUB_BUFFER_SIZE, std::bind(&can_driver::transition_handler_callback, this, std::placeholders::_1));

    // Assign values to stop frame
    stop_frame_.can_id = STATE_CAN_ID;
    stop_frame_.len = STOP_FRAME_LENGTH;
    stop_frame_.data[0] = static_cast<uint8_t>(triggering_board_state::STOP);
    // Assign values to calibrate IMU frame
    cal_imu_frame_.can_id = STATE_CAN_ID;
    cal_imu_frame_.len = CAL_IMU_FRAME_LENGTH;
    cal_imu_frame_.data[0] = static_cast<uint8_t>(triggering_board_state::CAL_IMU);
    // Assign values to calibrate camera frame
    cal_cam_frame_.can_id = STATE_CAN_ID;
    cal_cam_frame_.len = CAL_CAM_FRAME_LENGTH;
    cal_cam_frame_.data[0] = static_cast<uint8_t>(triggering_board_state::CAL_CAM);
    // Assign values to run frame
    run_frame_.can_id = STATE_CAN_ID;
    run_frame_.len = RUN_FRAME_LENGTH;
    run_frame_.data[0] = static_cast<uint8_t>(triggering_board_state::RUN);

    // Define socket timeout: 0.1 ms
    timeout_.tv_sec = 0;
    timeout_.tv_usec = 100000;

    // Log start of node
    RCLCPP_INFO(this->get_logger(), "can_driver started...");

}

// Destructor
can_driver::~can_driver() {

    transition_to_STOP();

}

void can_driver::transition_to_STOP() {

    // Check that state transition is valid
    if ((triggering_board_state_ != triggering_board_state::CAL_IMU) &&
        (triggering_board_state_ != triggering_board_state::CAL_CAM) &&
        (triggering_board_state_ != triggering_board_state::RUN)) {
            throw std::runtime_error("Transitioning to STOP from unspecified state is invalid");
    }

    // Perform state transition
    triggering_board_state_ = triggering_board_state::STOP;

    // Wait for can_reader_thread_ to finish executing
    if (can_reader_thread_.joinable()) {
        can_reader_thread_.join();
    }

    // Send triggering board command to stop running
    num_bytes_write_ = write(socket_, &stop_frame_, sizeof(struct canfd_frame));

    // Check that we didn't get an error
    if (num_bytes_write_ < 0) {
        perror("Error writing stop_frame_ to CAN Bus");
    }

    // Close the socket
    close(socket_);

    // Notify
    RCLCPP_INFO(this->get_logger(), "Triggering board stopped, CAN Bus closed");

}

void can_driver::transition_to_CAL_IMU() {

    // Check that state transition is valid
    // 1. Check that previous state was STOP
    if (triggering_board_state_ != triggering_board_state::STOP) {
        throw std::runtime_error("Transitioning to CAL_IMU from " + 
            state_to_string(triggering_board_state_) + " is invalid");
    }
    // Update imu rate
    imu_rate_ = this->get_parameter("imu_rate").as_int();
    // 2. Check that IMU rate is valid
    switch (imu_rate_) {
        case 26:
        case 52:
        case 104:
        case 208:
        case 416:
        case 833:
            break;
        default:
            throw std::runtime_error("IMU rate of " + 
            std::to_string(imu_rate_) + " Hz is invalid");
    }
    // Perform state transition
    triggering_board_state_ = triggering_board_state::CAL_IMU;

    // Configure CAN socket
    can_driver::configure_can_socket();

    // Add number of IMU calibration timestamps and IMU rate to CAN message
    std::memcpy(&cal_imu_frame_.data[1], &imu_calibration_samples_, sizeof(int32_t));
    std::memcpy(&cal_imu_frame_.data[5], &imu_rate_, sizeof(int32_t));
    
    // Send triggering board command to enter CAL_IMU state
    num_bytes_write_ = write(socket_, &cal_imu_frame_, sizeof(struct canfd_frame));

    // Check that we didn't get an error
    if (num_bytes_write_ < 0) {
        perror("Error writing cal_imu_frame_ to CAN Bus");
    }

    // Notify
    RCLCPP_INFO(this->get_logger(), "Number of IMU calibration samples = %d and IMU rate = %d", 
        imu_calibration_samples_, imu_rate_);

}

void can_driver::transition_to_CAL_CAM() {

    // Check that state transition is valid
    // 1. Check that previous state was CAL_IMU
    if (triggering_board_state_ != triggering_board_state::CAL_IMU) {
        throw std::runtime_error("Transitioning to CAL_CAM from " + 
            state_to_string(triggering_board_state_) + " is invalid");
    }
    // Update camera rate
    camera_rate_ = this->get_parameter("camera_rate").as_int();
    // 2. Check that camera rate is within bounds
    if (camera_rate_ < camera_rate_min_) {
        throw std::runtime_error("Camera rate is below min camera rate");
    }
    if (camera_rate_ > camera_rate_max_) {
        throw std::runtime_error("Camera rate is above max camera rate");
    }
    // 3. Check that number of camera calibration samples is nonzero
    if (camera_calibration_samples_ == 0) {
        throw std::runtime_error("Number of camera calibration samples is zero");
    }
    // Perform state transition
    triggering_board_state_ = triggering_board_state::CAL_CAM;

    // Add camera rate and number of camera calibration samples to CAN message
    std::memcpy(&cal_cam_frame_.data[1], &camera_calibration_samples_, sizeof(int32_t));
    std::memcpy(&cal_cam_frame_.data[5], &camera_rate_, sizeof(int32_t));

    // Send triggering board command to enter CAL_CAM state
    num_bytes_write_ = write(socket_, &cal_cam_frame_, sizeof(struct canfd_frame));

    // Check that we didn't get an error
    if (num_bytes_write_ < 0) {
        perror("Error writing cal_cam_frame_ to CAN Bus");
    }

    // Notify
    RCLCPP_INFO(this->get_logger(), "Number of camera calibration samples = %d and camera rate = %d FPS", 
        camera_calibration_samples_, camera_rate_);

}

void can_driver::transition_to_RUN() {

    // Check that state transition is valid
    // 1. Check that previous state was CAL_CAM
    if (triggering_board_state_ != triggering_board_state::CAL_CAM) {
        throw std::runtime_error("Transitioning to RUN from " + 
            state_to_string(triggering_board_state_) + " is invalid");
    }
    // Perform state transition
    triggering_board_state_ = triggering_board_state::RUN;        

    // Send triggering board command to enter RUN state
    num_bytes_write_ = write(socket_, &run_frame_, sizeof(struct canfd_frame));

    // Check that we didn't get an error
    if (num_bytes_write_ < 0) {
        perror("Error writing run_triggering_board_frame_ to CAN Bus");
    }

}

void can_driver::transition_handler_callback(const std_msgs::msg::UInt8::SharedPtr msg) {

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

void can_driver::camera_calibration_samples_callback(const std_msgs::msg::Int32::SharedPtr msg) {

    // Update number of camera calibration samples based on value published by orchestrator
    camera_calibration_samples_ = msg->data;

}

void can_driver::configure_can_socket() {
    
    // Define socket
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    
    // Set socket timeout
    setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout_, sizeof(timeout_));
    
    // Configure socket to use CAN FD
    if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_can_fd_, sizeof(enable_can_fd_)) < 0) {
        perror("Error setting socket options");
    }
    // Set interface name
    strcpy(interface_.ifr_name, can_socket_name_.c_str());
    
    // Get interface index
    ioctl(socket_, SIOCGIFINDEX, &interface_);
    
    // Set CAN socket address as a CAN address
    can_socket_address_.can_family = AF_CAN;
    
    // Set CAN socket index
    can_socket_address_.can_ifindex = interface_.ifr_ifindex;
    
    // Bind native socket (hardware) to can socket address (software)
    if (bind(socket_, (struct sockaddr*) &can_socket_address_, sizeof(can_socket_address_)) < 0) {
        throw std::runtime_error("Failed to bind socket");
    }

    // Start thread to read CAN bus
    can_reader_thread_ = std::thread(&can_driver::read_can, this);

}

void can_driver::read_can() {
    
    // Log start of CAN thread
    RCLCPP_INFO(this->get_logger(), "CAN thread started...");

    while ((triggering_board_state_ == triggering_board_state::CAL_IMU) ||
           (triggering_board_state_ == triggering_board_state::CAL_CAM) ||
           (triggering_board_state_ == triggering_board_state::RUN)) {

        // Read CAN frame
        num_bytes_read_ = read(socket_, &frame_, sizeof(struct canfd_frame));

        // Check that read() didn't timeout
        if (num_bytes_read_ < 0) {
            // WARNING: I think we only end up here if nothing is read but 
            // it's possible that we also end up here if read throws an error.
            // Be careful!
            continue;
        }

        // Check that we received a complete CAN frame
        if (num_bytes_read_ < sizeof(struct canfd_frame)) {
            throw std::runtime_error("Incomplete CAN frame");
        }

        // Check what type of CAN message we have received
        switch (frame_.can_id) {
            case STATE_CAN_ID:
                read_state_can_msg();
                break;
            case FINISHED_CAN_ID:
                read_finished_can_msg();
                break;
            case TIMESTAMPS_CAN_ID:
                read_timestamps_can_msg();
                break;
            case CAM_CAN_ID:
                read_cam_can_msg();
                break;
            case IMU_CAN_ID:
                read_imu_can_msg();
                break;
            default:
                throw std::runtime_error("CAN ID of received message not recognized");
        }

    }

    // Print stop of CAN thread
    RCLCPP_INFO(this->get_logger(), "CAN thread stopped");

}

void can_driver::read_state_can_msg() {

    // Get internal state that triggering board is in
    uint8_t raw_internal_state;
    std::memcpy(&raw_internal_state, &frame_.data[0], sizeof(uint8_t));
    triggering_board_state triggering_board_internal_state = 
        static_cast<triggering_board_state>(raw_internal_state);
    
    switch (triggering_board_internal_state) {
        
        case triggering_board_state::CAL_IMU:
            if (triggering_board_state_ != triggering_board_state::CAL_IMU) {
                throw std::runtime_error("ROS state is " + state_to_string(triggering_board_state_) +
                    " but triggering board internal state is " + state_to_string(triggering_board_internal_state));
            }
            break;

        case triggering_board_state::CAL_CAM:
            if (triggering_board_state_ != triggering_board_state::CAL_CAM) {
                throw std::runtime_error("ROS state is " + state_to_string(triggering_board_state_) +
                    " but triggering board internal state is " + state_to_string(triggering_board_internal_state));
            }
            break;

        case triggering_board_state::RUN:
            if (triggering_board_state_ != triggering_board_state::RUN) {
                throw std::runtime_error("ROS state is " + state_to_string(triggering_board_state_) +
                    " but triggering board internal state is " + state_to_string(triggering_board_internal_state));
            }
            break;

        default:
            throw std::runtime_error("Triggering board internal state is not valid");

    }

    // Notify
    RCLCPP_INFO(this->get_logger(), "Triggering board has entered %s", state_to_string(triggering_board_state_).c_str());
}

void can_driver::read_finished_can_msg() {

    // Extract message value
    uint8_t raw_message;
    std::memcpy(&raw_message, &frame_.data[0], sizeof(uint8_t));

    // Check which finished message was received
    switch (raw_message) {

        case FINISHED_IMU_CAL_MSG:
            
            // Check that message was sent from the correct state
            if (triggering_board_state_ != triggering_board_state::CAL_IMU) {
                throw std::runtime_error("Finished IMU calibration message sent from outside of CAL_IMU state");
            }
            
            // Confirm that we received the expected number of IMU calibration timestamps
            if (imu_calibration_counter_ != imu_calibration_samples_) {
                throw std::runtime_error(std::string("Only received ") + std::to_string(imu_calibration_counter_) +
                    std::string(" IMU calibration timestamps, but expected ") + std::to_string(imu_calibration_samples_));
            }

            // Reset counter
            imu_calibration_counter_ = 0;

            // Create message to publish
            imu_calibration_finished_msg_.data = static_cast<uint8_t>(triggering_board_state::CAL_CAM);

            // Wait for all the timestamps CAN messages to arrive before transitioning
            rclcpp::sleep_for(std::chrono::milliseconds(STATE_SWITCH_DELAY));

            // Request transition to CAM_CAL state
            state_publisher_->publish(imu_calibration_finished_msg_);

            break;

        case FINISHED_CAM_CAL_MSG:

            // Check that the message was sent from the correct state
            if (triggering_board_state_ != triggering_board_state::CAL_CAM) {
                throw std::runtime_error("Finished camera calibration message sent from outside of CAL_CAM state");
            }

            // Confirm that we received the expected number of camera timestamps
            if (camera_calibration_counter_ != camera_calibration_samples_) {
                throw std::runtime_error(std::string("Only received ") + std::to_string(camera_calibration_counter_) +
                    std::string(" camera timestamps, but expected ") + std::to_string(camera_calibration_samples_));
            }

            // Reset counter
            camera_calibration_counter_ = 0;

            // Create message to publish
            camera_calibration_finished_msg_.data = true;

            // Notify orchestrator that all camera timestamps have been received
            camera_calibration_finished_publisher_->publish(camera_calibration_finished_msg_);

            break;

        default:

            throw std::runtime_error("Unknown finished message received");

    }

}

void can_driver::read_timestamps_can_msg() {

    // Check that message was sent from correct state
    if (triggering_board_state_ != triggering_board_state::CAL_IMU) {
        throw std::runtime_error("IMU calibration timestamps message sent from outside of CAL_IMU state");
    }

    // Pass CAN bus frame to ROS2 message
    std::memcpy(&imu_timestamp_, &frame_.data[0], sizeof(uint32_t));
    std::memcpy(&mcu_timestamp_, &frame_.data[4], sizeof(uint32_t));
    calibration_timestamps_msg_.imu_timestamp = imu_timestamp_;
    calibration_timestamps_msg_.mcu_timestamp = mcu_timestamp_;

    // Publish timestamps
    calibration_timestamps_publisher_->publish(calibration_timestamps_msg_);

    // Update counter
    imu_calibration_counter_++;

}

void can_driver::read_cam_can_msg() {

    // Check that message was sent from correct state
    if ((triggering_board_state_ != triggering_board_state::CAL_CAM) ||
        (triggering_board_state_ != triggering_board_state::RUN)) {
        throw std::runtime_error("CAM CAN message sent from outside of CAL_CAM or RUN state");
    }

    // Pass CAN bus frame to ROS2 message
    std::memcpy(&cam_timestamp_, &frame_.data[0], sizeof(uint32_t));
    std::memcpy(&cam_frame_index_, &frame_.data[4], sizeof(uint32_t));
    cam_msg_.timestamp.sec = cam_timestamp_ / 1000;
    cam_msg_.timestamp.nanosec = (cam_timestamp_ % 1000) * 1000000;
    cam_msg_.index = cam_frame_index_;

    // Publish timestamp
    camera_timestamp_publisher_->publish(cam_msg_);

    // Update counter
    camera_calibration_counter_++;
    
}

void can_driver::read_imu_can_msg() {

    // Check that message was sent from correct state
    if ((triggering_board_state_ != triggering_board_state::CAL_CAM) ||
        (triggering_board_state_ != triggering_board_state::RUN)) {
        throw std::runtime_error("IMU CAN message sent from outside of CAL_CAM or RUN state");
    }
    
    // Pass CAN bus frame to ROS2 message
    std::memcpy(&acceleration_x_.as_bytes[0], &frame_.data[0], sizeof(float));
    std::memcpy(&acceleration_y_.as_bytes[0], &frame_.data[4], sizeof(float));
    std::memcpy(&acceleration_z_.as_bytes[0], &frame_.data[8], sizeof(float));
    std::memcpy(&angular_velocity_x_.as_bytes[0], &frame_.data[12], sizeof(float));
    std::memcpy(&angular_velocity_y_.as_bytes[0], &frame_.data[16], sizeof(float));
    std::memcpy(&angular_velocity_z_.as_bytes[0], &frame_.data[20], sizeof(float));
    std::memcpy(&timestamp_.as_bytes[0], &frame_.data[24], sizeof(uint32_t));

    // Unit conversion:
    // 1. Accelerometer: mg to m/s^2
    // 2. Gyroscope: deg/s to rad/s
    acceleration_x_.as_float = acceleration_x_.as_float*mg_to_ms2_;
    acceleration_y_.as_float = acceleration_y_.as_float*mg_to_ms2_;
    acceleration_z_.as_float = acceleration_z_.as_float*mg_to_ms2_;
    angular_velocity_x_.as_float = angular_velocity_x_.as_float*dps_to_rps_;
    angular_velocity_y_.as_float = angular_velocity_y_.as_float*dps_to_rps_;
    angular_velocity_z_.as_float = angular_velocity_z_.as_float*dps_to_rps_;

    // Pass readings to publisher message
    imu_msg_.linear_acceleration.x = acceleration_x_.as_float;
    imu_msg_.linear_acceleration.y = acceleration_y_.as_float;
    imu_msg_.linear_acceleration.z = acceleration_z_.as_float;
    imu_msg_.angular_velocity.x = angular_velocity_x_.as_float;
    imu_msg_.angular_velocity.y = angular_velocity_y_.as_float;
    imu_msg_.angular_velocity.z = angular_velocity_z_.as_float;
    imu_msg_.header.stamp.sec = timestamp_.as_uint32 / 1000;
    imu_msg_.header.stamp.nanosec = (timestamp_.as_uint32 % 1000) * 1000000;

    // Publish message
    imu_and_timestamp_publisher_->publish(imu_msg_);

}

// Main: code entry point
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<can_driver>());
    rclcpp::shutdown();
    return 0;
}