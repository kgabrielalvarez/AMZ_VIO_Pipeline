#include "camera_driver/camera_driver.hpp"

// Constructor
camera_driver::camera_driver() : Node("camera_driver") {

    // Initialize Publisher
    left_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("left_images", 10);
    right_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("right_images", 10);

    // Initialize Pylon runtime
    PylonInitialize();

    try {
        // Set up cameras
        configure_cameras();
        
        // Start getting images
        cameras_.StartGrabbing();

        // Log configuration parameters for left camera
        INodeMap& left_node_map = left_camera_.GetNodeMap();
        RCLCPP_INFO(this->get_logger(), "Left camera TriggerSelector: %s", CEnumParameter(left_node_map, "TriggerSelector").GetValue().c_str());
        RCLCPP_INFO(this->get_logger(), "Left camera TriggerMode: %s", CEnumParameter(left_node_map, "TriggerMode").GetValue().c_str());
        RCLCPP_INFO(this->get_logger(), "Left camera TriggerSource: %s", CEnumParameter(left_node_map, "TriggerSource").GetValue().c_str());
        RCLCPP_INFO(this->get_logger(), "Left camera TriggerActivation: %s", CEnumParameter(left_node_map, "TriggerActivation").GetValue().c_str());
        RCLCPP_INFO(this->get_logger(), "Left camera ExposureMode: %s", CEnumParameter(left_node_map, "ExposureMode").GetValue().c_str());
        CEnumParameter(left_node_map, "LineSelector").SetValue("Line3");
        RCLCPP_INFO(this->get_logger(), "Left camera line 3: %s", CEnumParameter(left_node_map, "LineSource").GetValue().c_str());

        // Print configuration parameters for right camera
        INodeMap& right_camera_map = right_camera_.GetNodeMap();
        RCLCPP_INFO(this->get_logger(), "Right camera TriggerSelector: %s", CEnumParameter(right_camera_map, "TriggerSelector").GetValue().c_str());
        RCLCPP_INFO(this->get_logger(), "Right camera TriggerMode: %s", CEnumParameter(right_camera_map, "TriggerMode").GetValue().c_str());
        RCLCPP_INFO(this->get_logger(), "Right camera TriggerSource: %s", CEnumParameter(right_camera_map, "TriggerSource").GetValue().c_str());
        RCLCPP_INFO(this->get_logger(), "Right camera TriggerActivation: %s", CEnumParameter(right_camera_map, "TriggerActivation").GetValue().c_str());
        RCLCPP_INFO(this->get_logger(), "Right camera ExposureMode: %s", CEnumParameter(right_camera_map, "ExposureMode").GetValue().c_str());

        // Start thread to read images
        image_reader_thread_ = std::thread(&camera_driver::read_images, this);
    }

    // Handle Pylon Specific Errors
    catch (const GenericException& error) {
        RCLCPP_ERROR(this->get_logger(), "Pylon Error: %s", error.GetDescription());
    }

    // Handle Custom Errors
    catch (const std::runtime_error& error) {
        RCLCPP_ERROR(this->get_logger(), error.what());
    }

}

// Destructor
camera_driver::~camera_driver() {
    RCLCPP_INFO(this->get_logger(), "Terminating Pylon");
    cameras_.StopGrabbing();
    // Wait for thread to stop running
    if (image_reader_thread_.joinable()) {
        image_reader_thread_.join();
    }
    PylonTerminate();
}

void camera_driver::read_images() {

    // Smart pointer to receive grabed frames
    // See https://docs.baslerweb.com/pylonapi/cpp/class_pylon_1_1_c_grab_result_data#function-gettimestamp
    // for the pointers public functions
    CGrabResultPtr ptrGrabResult;

    while (rclcpp::ok() && cameras_.IsGrabbing()) {

        try {
            // Get image
            cameras_.RetrieveResult(retrieve_result_timeout_, ptrGrabResult, TimeoutHandling_ThrowException);

            // Convert from Pylon to OpenCV format and publish
            convert_pylon_to_ros(ptrGrabResult);
        }

        // Handle Pylon Specific Errors
        catch (const GenericException& error) {
            RCLCPP_ERROR(this->get_logger(), "Pylon Error: %s", error.GetDescription());
            break;
        }

        // Handle Custom Errors
        catch (const std::runtime_error& error) {
            RCLCPP_ERROR(this->get_logger(), error.what());
            break;
        }

    }

}

void camera_driver::configure_cameras() {

    // Initialize transport layer factory, used to create, destroy, and enumerate transport layers and their devices
    CTlFactory& tl_factory = CTlFactory::GetInstance();

    // Add all detected cameras to devices_list_
    tl_factory.EnumerateDevices(devices_list_);

    // Initialize the camera array
    cameras_.Initialize(num_cameras_);

    // Check how many cameras have been detected
    if (devices_list_.size() < num_cameras_) {
        throw std::runtime_error("Detected " + std::to_string(devices_list_.size()) + " cameras but needed " + std::to_string(num_cameras_) + " cameras");
    }
    if (devices_list_.size() > num_cameras_) {
        RCLCPP_INFO(this->get_logger(), "Detected %d cameras but only using the first %d", devices_list_.size(), num_cameras_);
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Detected %d cameras", num_cameras_);
    }

    // Create camera instances
    for (size_t i = 0; i < num_cameras_; i++) {
        cameras_[i].Attach(tl_factory.CreateDevice(devices_list_[i]));
    }

    // Log camera serial numbers
    RCLCPP_INFO(this->get_logger(), "Left camera (MASTER) serial number: %s", left_camera_.GetDeviceInfo().GetSerialNumber().c_str());
    RCLCPP_INFO(this->get_logger(), "Right camera (SLAVE) serial number: %s", right_camera_.GetDeviceInfo().GetSerialNumber().c_str());

    // Configure left camera
    left_camera_.Open();
    INodeMap& left_node_map = left_camera_.GetNodeMap();

    // Configure input and output lines
    CEnumParameter(left_node_map, "LineSelector").SetValue("Line2");
    CEnumParameter(left_node_map, "LineMode").SetValue("Input");
    CEnumParameter(left_node_map, "LineSelector").SetValue("Line3");
    CEnumParameter(left_node_map, "LineMode").SetValue("Output");

    // Configure line 2 for hardware triggering:
    // 1. Set trigger to initiate frame start
    CEnumParameter(left_node_map, "TriggerSelector").SetValue("FrameStart");
    CEnumParameter(left_node_map, "TriggerMode").SetValue("On");
    CEnumParameter(left_node_map, "TriggerSource").SetValue("Line2");
    CEnumParameter(left_node_map, "TriggerActivation").SetValue("RisingEdge");
    // 2. Set exposure mode to "TriggerWidth"
    CEnumParameter(left_node_map, "ExposureMode").SetValue("TriggerWidth");

    // Configure line 3 for exposure active signal:
    CEnumParameter(left_node_map, "LineSelector").SetValue("Line3");
    CEnumParameter(left_node_map, "LineSource").SetValue("ExposureActive");

    // Configure right camera
    right_camera_.Open();
    INodeMap& right_node_map = right_camera_.GetNodeMap();

    // Configure input line
    CEnumParameter(right_node_map, "LineSelector").SetValue("Line2");
    CEnumParameter(right_node_map, "LineMode").SetValue("Input");

    // Configure line 2 for hardware triggering:
    // 1. Set trigger to initiate frame start
    CEnumParameter(right_node_map, "TriggerSelector").SetValue("FrameStart");
    CEnumParameter(right_node_map, "TriggerMode").SetValue("On");
    CEnumParameter(right_node_map, "TriggerSource").SetValue("Line2");
    CEnumParameter(right_node_map, "TriggerActivation").SetValue("RisingEdge");
    // 2. Set exposure mode to "TriggerWidth"
    CEnumParameter(right_node_map, "ExposureMode").SetValue("TriggerWidth");

}

void camera_driver::convert_pylon_to_ros(const CGrabResultPtr& image_ptr) {

    // Check that image was succesfully grabbed
    if(!image_ptr->GrabSucceeded()) {
        throw std::runtime_error("Failed to grab image");
    }

    // Get camera index
    intptr_t camera_index = image_ptr->GetCameraContext();

    // Create OpenCV image to store image
    cv_bridge::CvImage cv_image;

    // Check and set pixel type
    EPixelType pixel_type = image_ptr->GetPixelType();
    if (pixel_type != pixel_type_pylon_) {
        throw std::runtime_error("Camera " + std::to_string(camera_index) + " uses incorrect pixel type: " + std::to_string(pixel_type));
    }
    cv_image.encoding = pixel_type_cv_;

    // TO-DO: assign coordinate system
    // cv_image.header.frame_id = ;

    // Set timestamp corresponding to current time
    cv_image.header.stamp = this->get_clock()->now();

    // Get number of bytes per row
    size_t bytes_per_row;
    if (!image_ptr->GetStride(bytes_per_row)) {
        throw std::runtime_error("Camera " + std::to_string(camera_index) + " failed to get stride");
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
        throw std::runtime_error("Camera index " + std::to_string(camera_index) + " does not match 0 or 1");
    }

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_driver>());
    rclcpp::shutdown();
    return 0;
}