#include "camera_driver/camera_driver.hpp"

// Constructor
camera_driver::camera_driver() : Node("camera_driver") {

    // Declare parameters
    this->declare_parameter("min_exposure_time", 0.0);
    this->declare_parameter("max_exposure_time", 0.0);

    // Initialize publishers and subscriber
    left_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("left_images", 10);
    right_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("right_images", 10);
    state_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>("triggering_board_state", 
        10, std::bind(&camera_driver::transition_handler_callback, this, std::placeholders::_1));

    // Log start of node
    RCLCPP_INFO(this->get_logger(), "camera_driver started...");

}

// Destructor
camera_driver::~camera_driver() {

    transition_to_STOP();

}

void camera_driver::transition_to_STOP() {

    // Check that state transition is valid
    if ((triggering_board_state_ != triggering_board_state::CAL_IMU) &&
        (triggering_board_state_ != triggering_board_state::CAL_CAM) &&
        (triggering_board_state_ != triggering_board_state::RUN)) {
            throw std::runtime_error("Transitioning to STOP from unspecified state is invalid");
    }
    // Perform state transition
    triggering_board_state_ = triggering_board_state::STOP;

    // Stop Pylon
    cameras_.StopGrabbing();
    // Wait for thread to stop running
    if (image_reader_thread_.joinable()) {
        image_reader_thread_.join();
    }
    PylonTerminate();

    // Notify
    RCLCPP_INFO(this->get_logger(), "Pylon has been terminated");

}

void camera_driver::transition_to_CAL_IMU() {

    // Check that state transition is valid
    // 1. Check that previous state was STOP
    if (triggering_board_state_ != triggering_board_state::STOP) {
        throw std::runtime_error("Transitioning to CAL_IMU from " + 
            state_to_string(triggering_board_state_) + " is invalid");
    }
    // Perform state transition
    triggering_board_state_ = triggering_board_state::CAL_IMU;

    // Do nothing in this state

    // Notify
    RCLCPP_INFO(this->get_logger(), "camera_driver node has entered %s", state_to_string(triggering_board_state_).c_str());

}

void camera_driver::transition_to_CAL_CAM() {

    // Check that state transition is valid
    // 1. Check that previous state was CAL_IMU
    if (triggering_board_state_ != triggering_board_state::CAL_IMU) {
        throw std::runtime_error("Transitioning to CAL_CAM from " + 
            state_to_string(triggering_board_state_) + " is invalid");
    }
    // Perform state transition
    triggering_board_state_ = triggering_board_state::CAL_CAM;

    // Initialize Pylon runtime
    PylonInitialize();

    // Configure cameras
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

        // Log configuration parameters for right camera
        INodeMap& right_camera_map = right_camera_.GetNodeMap();
        RCLCPP_INFO(this->get_logger(), "Right camera TriggerSelector: %s", CEnumParameter(right_camera_map, "TriggerSelector").GetValue().c_str());
        RCLCPP_INFO(this->get_logger(), "Right camera TriggerMode: %s", CEnumParameter(right_camera_map, "TriggerMode").GetValue().c_str());
        RCLCPP_INFO(this->get_logger(), "Right camera TriggerSource: %s", CEnumParameter(right_camera_map, "TriggerSource").GetValue().c_str());
        RCLCPP_INFO(this->get_logger(), "Right camera TriggerActivation: %s", CEnumParameter(right_camera_map, "TriggerActivation").GetValue().c_str());
        RCLCPP_INFO(this->get_logger(), "Right camera ExposureMode: %s", CEnumParameter(right_camera_map, "ExposureMode").GetValue().c_str());

        // Configure left camera (master) to use constant exposure
        CEnumParameter(left_node_map, "ExposureAuto").SetValue("Off");
        CEnumParameter(left_node_map, "ExposureTimeMode").SetValue("Common");
        CFloatParameter(left_node_map, "ExposureTime").SetValue(constant_exposure_time_);

        // Log exposure configuration parameters
        RCLCPP_INFO(this->get_logger(), "Left camera ExposureAuto: %s", CEnumParameter(left_node_map, "ExposureAuto").GetValue().c_str());
        RCLCPP_INFO(this->get_logger(), "Left camera ExposureTimeMode: %s", CEnumParameter(left_node_map, "ExposureTimeMode").GetValue().c_str());
        RCLCPP_INFO(this->get_logger(), "Left camera ExposureTime: %f us", CFloatParameter(left_node_map, "ExposureTime").GetValue());

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

    // Notify
    RCLCPP_INFO(this->get_logger(), "camera_driver node has entered %s", 
        state_to_string(triggering_board_state_).c_str());

}

void camera_driver::transition_to_RUN() {

    // Check that state transition is valid
    // 1. Check that previous state was CAL_CAM
    if (triggering_board_state_ != triggering_board_state::CAL_CAM) {
        throw std::runtime_error("Transitioning to RUN from " + 
            state_to_string(triggering_board_state_) + " is invalid");
    }
    // Perform state transition
    triggering_board_state_ = triggering_board_state::RUN;

    // Configure cameras
    try {
        // Configure left camera (master) to use autoexposure
        INodeMap& left_node_map = left_camera_.GetNodeMap();
        CEnumParameter(left_node_map, "ExposureAuto").SetValue("On");
        
        // Set min and max exposure times
        min_exposure_time_ = this->get_parameter("min_exposure_time").as_double();
        max_exposure_time_ = this->get_parameter("max_exposure_time").as_double();
        // TO-DO update to use parameters in YAML file and get rid of the two lines below!!!
        min_exposure_time_ = CFloatParameter(left_node_map, "AutoExposureTimeLowerLimit").GetMin();
        max_exposure_time_ = CFloatParameter(left_node_map, "AutoExposureTimeUpperLimit").GetMax();
        CFloatParameter(left_node_map, "AutoExposureTimeLowerLimit").SetValue(min_exposure_time_);
        CFloatParameter(left_node_map, "AutoExposureTimeUpperLimit").SetValue(max_exposure_time_);

        // Log configuration parameters
        RCLCPP_INFO(this->get_logger(), "Left camera ExposureAuto: %s", 
            CEnumParameter(left_node_map, "ExposureAuto").GetValue().c_str());
        RCLCPP_INFO(this->get_logger(), "Left camera AutoExposureTimeLowerLimit: %f us", 
            CEnumParameter(left_node_map, "AutoExposureTimeLowerLimit").GetValue());
        RCLCPP_INFO(this->get_logger(), "Left camera AutoExposureTimeUpperLimit: %f us", 
            CEnumParameter(left_node_map, "AutoExposureTimeUpperLimit").GetValue());
    }

    // Handle Pylon Specific Errors
    catch (const GenericException& error) {
        RCLCPP_ERROR(this->get_logger(), "Pylon Error: %s", error.GetDescription());
    }

    // Handle Custom Errors
    catch (const std::runtime_error& error) {
        RCLCPP_ERROR(this->get_logger(), error.what());
    }

    // Notify
    RCLCPP_INFO(this->get_logger(), "camera_driver node has entered %s", 
        state_to_string(triggering_board_state_).c_str());
    
}

void camera_driver::transition_handler_callback(const std_msgs::msg::UInt8::SharedPtr msg) {

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
    // 2. Set exposure mode to "Timed" since this is the "Master" camera
    CEnumParameter(left_node_map, "ExposureMode").SetValue("Timed");

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
    // 2. Set exposure mode to "TriggerWidth" since this is the "Slave" camera
    CEnumParameter(right_node_map, "ExposureMode").SetValue("TriggerWidth");

}

void camera_driver::read_images() {

    // Smart pointer to receive grabed frames
    // See https://docs.baslerweb.com/pylonapi/cpp/class_pylon_1_1_c_grab_result_data#function-gettimestamp
    // for the pointers public functions
    CGrabResultPtr ptrGrabResult;

    while (((triggering_board_state_ == triggering_board_state::CAL_IMU) ||
            (triggering_board_state_ == triggering_board_state::CAL_CAM) ||
            (triggering_board_state_ == triggering_board_state::RUN)) && 
           cameras_.IsGrabbing()) {

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

// Main: code entry point
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_driver>());
    rclcpp::shutdown();
    return 0;
}