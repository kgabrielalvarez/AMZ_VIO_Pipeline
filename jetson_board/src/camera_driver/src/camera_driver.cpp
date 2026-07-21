#include "camera_driver/camera_driver.hpp"

// Constructor
camera_driver::camera_driver() : Node("camera_driver") {

    // Initialize Pylon runtime
    PylonInitialize();

    try {
        // Create an instant camera object with the first camera device found
        CInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice() );
        // Print camera found
        std::cout << "Using device: " << camera.GetDeviceInfo().GetModelName() << std::endl << std::endl;
    }
    catch (const GenericException& e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl 
        << e.GetDescription() << std::endl;
    }

    std::cout << "Ran successfully!" << std::endl;

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_driver>());
    rclcpp::shutdown();
    return 0;
}