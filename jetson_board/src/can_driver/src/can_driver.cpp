#include "can_driver/can_driver.hpp"

// Constructor
can_driver::can_driver() : Node("can_driver") {

    // Declare parameters
    this->declare_parameter("camera_rate", 0); // [FPS]
    this->declare_parameter("imu_rate", 0); // [Hz]

    // Initialize publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_and_timestamps", 10);
    // Initialize subscriber
    subscriber_ = this->create_subscription<std_msgs::msg::UInt8>("triggering_board_state", 
        10, std::bind(&can_driver::subscriber_callback, this, std::placeholders::_1));

    // Assign values to stop frame
    stop_triggering_board_frame_.can_id = CAN_ID;
    stop_triggering_board_frame_.len = STOP_FRAME_LENGTH;
    stop_triggering_board_frame_.data[0] = static_cast<uint8_t>(triggering_board_state::STOP);
    // Assign values to cal frame
    cal_triggering_board_frame_.can_id = CAN_ID;
    cal_triggering_board_frame_.len = CAL_FRAME_LENGTH;
    cal_triggering_board_frame_.data[0] = static_cast<uint8_t>(triggering_board_state::CAL);
    // Assign values to run frame
    run_triggering_board_frame_.can_id = CAN_ID;
    run_triggering_board_frame_.len = RUN_FRAME_LENGTH;
    run_triggering_board_frame_.data[0] = static_cast<uint8_t>(triggering_board_state::RUN);

    // Define socket timeout
    timeout_.tv_sec = 0;
    timeout_.tv_usec = 100000;

    // Print start of node
    RCLCPP_INFO(this->get_logger(), "can_driver started...");

}

// Destructor
can_driver::~can_driver() {

    triggering_board_state_ = triggering_board_state::STOP;
    this->stop_triggering_board();

}

void can_driver::read_can() {
    
    // Print start of CAN thread
    RCLCPP_INFO(this->get_logger(), "CAN thread started...");

    while ((triggering_board_state_ == triggering_board_state::CAL) ||
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

        // Convert uint8_t to float
        std::memcpy(&acceleration_x_.as_bytes[0], &frame_.data[0], 4);
        std::memcpy(&acceleration_y_.as_bytes[0], &frame_.data[4], 4);
        std::memcpy(&acceleration_z_.as_bytes[0], &frame_.data[8], 4);
        std::memcpy(&angular_velocity_x_.as_bytes[0], &frame_.data[12], 4);
        std::memcpy(&angular_velocity_y_.as_bytes[0], &frame_.data[16], 4);
        std::memcpy(&angular_velocity_z_.as_bytes[0], &frame_.data[20], 4);
        std::memcpy(&timestamp_.as_bytes[0], &frame_.data[24], 4);

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
        imu_message_.linear_acceleration.x = acceleration_x_.as_float;
        imu_message_.linear_acceleration.y = acceleration_y_.as_float;
        imu_message_.linear_acceleration.z = acceleration_z_.as_float;
        imu_message_.angular_velocity.x = angular_velocity_x_.as_float;
        imu_message_.angular_velocity.y = angular_velocity_y_.as_float;
        imu_message_.angular_velocity.z = angular_velocity_z_.as_float;
        imu_message_.header.stamp.sec = timestamp_.as_uint32 / 1000;
        imu_message_.header.stamp.nanosec = (timestamp_.as_uint32 % 1000) * 1000000;

        // Publish message
        publisher_->publish(imu_message_);

    }

    // Print stop of CAN thread
    RCLCPP_INFO(this->get_logger(), "CAN thread stopped");

}

void can_driver::start_and_calibrate_triggering_board() {

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

    // Add IMU and camera rate to CAN message
    std::memcpy(&cal_triggering_board_frame_.data[1], &imu_rate_, INT_LENGTH);
    std::memcpy(&cal_triggering_board_frame_.data[5], &camera_calibration_rate_, INT_LENGTH);
    
    // Send triggering board command to enter calibration state
    num_bytes_write_ = write(socket_, &cal_triggering_board_frame_, sizeof(struct canfd_frame));

    // Check that we didn't get an error
    if (num_bytes_write_ < 0) {
        perror("Error writing cal_triggering_board_frame_ to CAN Bus");
    }

    // TO-DO: confirm that STM32 started publishing

    // Notify
    RCLCPP_INFO(this->get_logger(), "Triggering board entering calibration state");
    RCLCPP_INFO(this->get_logger(), "IMU rate = %d Hz and camera rate = %d FPS", imu_rate_, camera_calibration_rate_);

}

void can_driver::run_triggering_board() {

    // Add IMU and camera rate to CAN message
    std::memcpy(&run_triggering_board_frame_.data[1], &imu_rate_, INT_LENGTH);
    std::memcpy(&run_triggering_board_frame_.data[5], &camera_rate_, INT_LENGTH);

    // Send triggering board command to enter run state
    num_bytes_write_ = write(socket_, &run_triggering_board_frame_, sizeof(struct canfd_frame));

    // Check that we didn't get an error
    if (num_bytes_write_ < 0) {
        perror("Error writing run_triggering_board_frame_ to CAN Bus");
    }

    // Notify
    RCLCPP_INFO(this->get_logger(), "Triggering board entering run state");
    RCLCPP_INFO(this->get_logger(), "IMU rate = %d Hz and camera rate = %d FPS", imu_rate_, camera_rate_);

}

void can_driver::stop_triggering_board() {

    // Wait for can_reader_thread_ to finish executing
    if (can_reader_thread_.joinable()) {
        can_reader_thread_.join();
    }

    // Send triggering board command to stop running
    num_bytes_write_ = write(socket_, &stop_triggering_board_frame_, sizeof(struct canfd_frame));

    // Check that we didn't get an error
    if (num_bytes_write_ < 0) {
        perror("Error writing stop_triggering_board_frame_ to CAN Bus");
    }

    // TO-DO: confirm that STM32 stopped publishing

    // Close the socket
    close(socket_);

    // Notify
    RCLCPP_INFO(this->get_logger(), "Triggering board stopped, CAN Bus closed");

}

void can_driver::subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg) {

    // Save previous state
    triggering_board_state triggering_board_previous_state = triggering_board_state_;

    // Update current state
    triggering_board_state_ = static_cast<triggering_board_state>(msg->data);

    // Transition to commanded state
    switch(triggering_board_state_) {

        case triggering_board_state::STOP:
            this->stop_triggering_board();
            break;

        case triggering_board_state::CAL:
            // Update IMU rate
            imu_rate_ = this->get_parameter("imu_rate").as_int();
            // Check that IMU rate is within bounds
            if (imu_rate_ > imu_rate_max_) {
                throw std::runtime_error("IMU rate is above max IMU rate");
            }
            this->start_and_calibrate_triggering_board();
            break;

        case triggering_board_state::RUN:
            // Check that triggering board was started before running
            if (triggering_board_previous_state != triggering_board_state::CAL) {
                throw std::runtime_error("Trying to run triggering board but it wasn't in CAL state");
            }
            // Update IMU and camera rates
            imu_rate_ = this->get_parameter("imu_rate").as_int();
            camera_rate_ = this->get_parameter("camera_rate").as_int();
            // Check that IMU rate is within bounds
            if (imu_rate_ > imu_rate_max_) {
                throw std::runtime_error("IMU rate is above max IMU rate");
            }
            // Check that camera rate is within bounds
            if (camera_rate_ < camera_rate_min_) {
                throw std::runtime_error("Camera rate is below min camera rate");
            }
            if (camera_rate_ > camera_rate_max_) {
                throw std::runtime_error("Camera rate is above max camera rate");
            }
            // Check that IMU rate is a multiple of camera rate
            if ((imu_rate_ % camera_rate_) != 0) {
                throw std::runtime_error("IMU rate is not a multiple of camera rate");
            }
            this->run_triggering_board();
            break;

        default:
            throw std::runtime_error("triggering_board_state_ not valid");
            break;

    }

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<can_driver>());
    rclcpp::shutdown();
    return 0;
}