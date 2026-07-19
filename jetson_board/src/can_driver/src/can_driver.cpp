#include "can_driver/can_driver.hpp"

// Constructor
can_driver::can_driver() : Node("can_driver") {

    // Initialize Publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_and_timestamps", 10);
    // Initialize Subscriber
    subscriber_ = this->create_subscription<std_msgs::msg::Bool>("triggering_board_active", 
        10, std::bind(&can_driver::subscriber_callback, this, std::placeholders::_1));

}

void can_driver::start_can() {

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

    // Enable flag for can_reader_thread_ to run continuously
    run_triggering_board_ = true;
    // Start thread to read CAN bus
    can_reader_thread_ = std::thread(&can_driver::read_can, this);
    // Allows thread to run independently
    can_reader_thread_.detach();

    // TO-DO: move this to subscriber callback
    this->start_triggering_board();
}

void can_driver::read_can() {
    while (run_triggering_board_) {

        // Read CAN frame
        num_bytes_read_ = read(socket_, &frame_, sizeof(struct can_frame));

        // Check that we didn't get an error
        if (num_bytes_read_ < 0) {
            perror("Error reading CAN Bus");
            break;
        }

        // Check that we received a complete CAN frame
        if (num_bytes_read_ < sizeof(struct can_frame)) {
            fprintf(stderr, "Error: incomplete CAN frame\n");
            break;
        }

        // Decode: TO-DO

        // Publish: TO-DO
        printf("Data bytes 0x%x, 0x%x, 0x%x \n", frame_.data[0], frame_.data[1], frame_.data[2]);

    }
}

void can_driver::stop_can() {
    // TO-DO
}

void can_driver::start_triggering_board() {

    this->start_can();
    // Send triggering board command to start running: TO-DO

}

void can_driver::stop_triggering_board() {
    // TO-DO
}

void can_driver::subscriber_callback(const std_msgs::msg::Bool msg) {
    // TO-DO
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<can_driver>());
    rclcpp::shutdown();
    return 0;
}