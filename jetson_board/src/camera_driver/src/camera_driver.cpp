#include "camera_driver/camera_driver.hpp"

// Constructor
camera_driver::camera_driver() : Node("camera_driver") {

    // Initialize Publisher
    left_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("left_images", 10);
    right_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("right_images", 10);

    // Initialize Pylon runtime
    PylonInitialize();

    try {

        // Initialize transport layer factory, used to create, destroy, and enumerate transport layers and their devices
        CTlFactory& tl_factory = CTlFactory::GetInstance();

        // Add all detected cameras to devices_list_
        tl_factory.EnumerateDevices(devices_list_);

        // Initialize the camera array
        cameras_.Initialize(num_cameras_);

        // Check how many cameras have been detected
        if (devices_list_.size() < num_cameras_) {
            RCLCPP_ERROR(this->get_logger(), "Detected %d cameras but need %d cameras", devices_list_.size(), num_cameras_);
        }
        else {
            if (devices_list_.size() == num_cameras_) {
                RCLCPP_INFO(this->get_logger(), "Detected %d cameras", num_cameras_);
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Detected %d cameras but only using the first %d", devices_list_.size(), num_cameras_);
            }

            // Create camera instances
            for (size_t i = 0; i < num_cameras_; i++) {
                cameras_[i].Attach(tl_factory.CreateDevice(devices_list_[i]));
            }
            RCLCPP_INFO(this->get_logger(), "Left camera serial number: %s", cameras_[0].GetDeviceInfo().GetSerialNumber().c_str());
            RCLCPP_INFO(this->get_logger(), "Right camera serial number: %s", cameras_[1].GetDeviceInfo().GetSerialNumber().c_str());
        }

        // Check that all parameters are set correctly
        // check_parameters();

        // Start frame grabing
        cameras_.StartGrabbing();

        // Smart pointer to receive grabed frames
        // See https://docs.baslerweb.com/pylonapi/cpp/class_pylon_1_1_c_grab_result_data#function-gettimestamp
        // for the pointers public functions
        CGrabResultPtr ptrGrabResult;

        // Grab the images
        for (int i = 0; i < 5000 && cameras_.IsGrabbing(); i++) {
            
            cameras_.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);

            convert_pylon_to_ros(ptrGrabResult);

        }

    }
    catch (const GenericException& e) {
        RCLCPP_ERROR(this->get_logger(), "An exception occurred: %s", e.GetDescription());
    }

}

// Destructor
camera_driver::~camera_driver() {
    RCLCPP_INFO(this->get_logger(), "Terminating Pylon");
    cameras_.StopGrabbing();
    PylonTerminate();
}

void camera_driver::check_parameters() {

    // See: https://docs.baslerweb.com/pylonapi/cpp/sample_code#parametrizecamera_genericparameteraccess
    // for example code
    for (int i = 0; i < num_cameras_; i++) {
        cameras_[i].Open();
        INodeMap& node_map = cameras_[i].GetNodeMap();
        CEnumParameter pixel_format(node_map, "PixelFormat");
        std::cout << "Camera " << i << " pixel format: " << pixel_format.GetValue() << std::endl;
        cameras_[i].Close();

        // TO-DO the above needs to be fixed --> need to properly check through all the parameters and make sure they are actually what we exepct them to be.
    }

}

void camera_driver::convert_pylon_to_ros(const CGrabResultPtr& image_ptr) {

    // Check that image was succesfully grabbed
    if(!image_ptr->GrabSucceeded()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to grab image");
        return;
    }

    // Get camera index
    intptr_t camera_index = image_ptr->GetCameraContext();

    // Create OpenCV image to store image
    cv_bridge::CvImage cv_image;

    // Check and set pixel type
    EPixelType pixel_type = image_ptr->GetPixelType();
    if (pixel_type != pixel_type_pylon_) {
        RCLCPP_ERROR(this->get_logger(), "CAMERA %d: incorrect pixel type (%d)", camera_index, pixel_type);
        return;
    }
    cv_image.encoding = pixel_type_cv_;

    // TO-DO: assign coordinate system
    // cv_image.header.frame_id = ;

    // Set timestamp corresponding to current time
    cv_image.header.stamp = this->get_clock()->now();

    // Get number of bytes per row
    size_t bytes_per_row;
    if (!image_ptr->GetStride(bytes_per_row)) {
        RCLCPP_ERROR(this->get_logger(), "CAMERA %d: failed to get stride", camera_index);
        return;
    }

    // Convert pylon image to OpenCV image
    // See https://docs.opencv.org/doc/doxygen/html/d3/d63/classcv_1_1Mat.html?utm_source=chatgpt.com
    // for OpenCV documentation
    cv_image.image = cv::Mat(image_ptr->GetHeight(),
                             image_ptr->GetWidth(),
                             cv_mat_type_,
                             image_ptr->GetBuffer(),
                             bytes_per_row);

    // Publish OpenCV image
    if (camera_index == 0) {
        left_image_publisher_->publish(*cv_image.toImageMsg());
        RCLCPP_INFO(this->get_logger(), "Published left image");
    }
    else if (camera_index == 1) {
        right_image_publisher_->publish(*cv_image.toImageMsg());
        RCLCPP_INFO(this->get_logger(), "Published right image");
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Camera index is %d, does not match 0 or 1", camera_index);
    }

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_driver>());
    rclcpp::shutdown();
    return 0;
}