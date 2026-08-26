// Code based on:
// 1. https://ja.docs.baslerweb.com/pylonapi/cpp/sample_code#utility_grabvideo
// Intended hardware:
// 1. Camera model: https://www.baslerweb.com/en/shop/a2a1920-168mgc/
// 2. GMSL Adapter board: https://www.baslerweb.com/en/shop/gmsl-adapter-kit-for-orin-nano-dev-kit/

#ifndef camera_driver_h
#define camera_driver_h

// Include Pylon specific libraries
#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// Include ROS2 libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "orchestrator/orchestrator_utils.hpp"

// Macros
// Cameras
#define left_camera_ cameras_[0]
#define right_camera_ cameras_[1]

// Include namespaces
using namespace Pylon;
using namespace GenApi; // From the GenICam standard

// Driver to connect camera to Jetson over GMSL adpater board
class camera_driver : public rclcpp::Node {

    public:

        // Constructor
        camera_driver();

        //
        ~camera_driver();

    private:

        // Variables to read cameras
        DeviceInfoList_t devices_list_;
        int num_cameras_ = 2;
        CInstantCameraArray cameras_;

        // Camera parameters
        EPixelType pixel_type_pylon_ = PixelType_BayerRG8;
        std::string pixel_type_cv_ = sensor_msgs::image_encodings::BAYER_RGGB8;
        int cv_mat_type_ = CV_8UC1;
        int retrieve_result_timeout_ = 1000; // ms

        // Thread to read images
        std::thread image_reader_thread_;

        // Triggering board state and requested state to transition to
        triggering_board_state triggering_board_state_;
        triggering_board_state state_to_transition_to_;

        // Constant exposure time
        double constant_exposure_time_ = 3500.0; // [us]

        // Autoexposure min and max exposure times
        double min_exposure_time_; // [us] User defined in YAML file
        double max_exposure_time_; // [us] User defined in YAML file

        // Publishers and subscriber
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_publisher_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_subscriber_;

        // Methods: state transitions
        void transition_to_STOP();
        void transition_to_CAL_IMU();
        void transition_to_CAL_CAM();
        void transition_to_RUN();

        // Methods: miscellaneous
        void transition_handler_callback(const std_msgs::msg::UInt8::SharedPtr msg);
        void configure_cameras();
        void read_images();
        void convert_pylon_to_ros(const CGrabResultPtr& image_ptr);

};

#endif