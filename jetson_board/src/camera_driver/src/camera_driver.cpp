#include "camera_driver/camera_driver.hpp"

// Constructor
camera_driver::camera_driver() : Node("camera_driver") {

    // Initialize Pylon runtime
    PylonInitialize();

    // Initialize transport layer factory, used to create, destroy, and enumerate transport layers and their devices
    CTlFactory& tl_factory = CTlFactory::GetInstance();

    // Initialize list to store transport layer devices
    DeviceInfoList_t devices_list;

    // Add all detected cameras to list
    tl_factory.EnumerateDevices(devices_list);

    // Check how many cameras have been detected
    if (devices_list.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Detected %d cameras but need at least 2 cameras\n", devices_list.size());
    }
    else {
        if (devices_list.size() == 2) {
            RCLCPP_INFO(this->get_logger(), "Detected 2 cameras\n");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Detected %d cameras but only using the first 2\n", devices_list.size());
        }

        // Create camera instances
        CInstantCamera camera_left(tl_factory.CreateDevice(devices_list[0]));
        CInstantCamera camera_right(tl_factory.CreateDevice(devices_list[1]));
        RCLCPP_INFO(this->get_logger(), "Left camera serial number: %s", camera_left.GetDeviceInfo().GetSerialNumber().c_str());
        RCLCPP_INFO(this->get_logger(), "Right camera serial number: %s", camera_right.GetDeviceInfo().GetSerialNumber().c_str());
    }

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_driver>());
    rclcpp::shutdown();
    return 0;
}