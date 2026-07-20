#include "can_driver/can_driver.hpp"

// Constructor
can_driver::can_driver() : Node("can_driver") {

    // Initialize Publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_and_timestamps", 10);
    // Initialize Subscriber
    subscriber_ = this->create_subscription<std_msgs::msg::Bool>("triggering_board_active", 
        10, std::bind(&can_driver::subscriber_callback, this, std::placeholders::_1));

    // Assign values to start and stop frames
    start_triggering_board_frame_.can_id = 0x001;
    start_triggering_board_frame_.len = 1;
    start_triggering_board_frame_.data[0] = 0x01;
    stop_triggering_board_frame_.can_id = 0x001;
    stop_triggering_board_frame_.len = 1;
    stop_triggering_board_frame_.data[0] = 0x00;

    // Print start of node
    RCLCPP_INFO(this->get_logger(), "can_driver started...");

}

// Destructor
can_driver::~can_driver() {

    run_triggering_board_.store(false);
    this->stop_triggering_board();

}

void can_driver::read_can() {
    
    // Print start of CAN thread
    RCLCPP_INFO(this->get_logger(), "CAN thread started...");

    while (run_triggering_board_.load()) {

        // Read CAN frame
        num_bytes_read_ = read(socket_, &frame_, sizeof(struct canfd_frame));

        // Check that we didn't get an error
        if (num_bytes_read_ < 0) {
            perror("Error reading CAN Bus");
        }

        // Check that we received a complete CAN frame
        if (num_bytes_read_ < sizeof(struct canfd_frame)) {
            RCLCPP_ERROR(this->get_logger(), "Error: incomplete CAN frame\n");
        }

        // Convert uint8_t to float
        std::memcpy(&acceleration_x_.as_bytes[0], &frame_.data[0], 4);
        std::memcpy(&acceleration_y_.as_bytes[0], &frame_.data[4], 4);
        std::memcpy(&acceleration_z_.as_bytes[0], &frame_.data[8], 4);
        std::memcpy(&angular_rate_x_.as_bytes[0], &frame_.data[12], 4);
        std::memcpy(&angular_rate_y_.as_bytes[0], &frame_.data[16], 4);
        std::memcpy(&angular_rate_z_.as_bytes[0], &frame_.data[20], 4);
        std::memcpy(&timestamp_.as_bytes[0], &frame_.data[4], 4);

        // Publish: TO-DO
        printf("Acceleration-x = %f.3 \n", acceleration_x_.as_float);

    }

    // Print stop of CAN thread
    RCLCPP_INFO(this->get_logger(), "CAN thread stopped");

}

void can_driver::start_triggering_board() {

    // Define socket
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
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
        RCLCPP_ERROR(this->get_logger(), "Error in binding socket");
    }

    // Start thread to read CAN bus
    can_reader_thread_ = std::thread(&can_driver::read_can, this);
    
    // Send triggering board command to start running
    num_bytes_write_ = write(socket_, &start_triggering_board_frame_, sizeof(struct canfd_frame));

    // Check that we didn't get an error
    if (num_bytes_write_ < 0) {
        perror("Error writing start_triggering_board_frame_ to CAN Bus");
    }

    // TO-DO: confirm that STM32 started publishing

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

    // Notify t
    RCLCPP_INFO(this->get_logger(), "Triggering board stopped, CAN Bus closed");

}

void can_driver::subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg) {

    // Enable or disable flag for can_reader_thread_ to run continuously
    run_triggering_board_.store(msg->data);

    if (run_triggering_board_.load()) {
        this->start_triggering_board();
    }
    else {
        this->stop_triggering_board();
    }

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<can_driver>());
    rclcpp::shutdown();
    return 0;
}