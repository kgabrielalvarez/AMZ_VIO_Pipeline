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

// Macros
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

        // Publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_publisher_;

        // Methods
        void configure_cameras();
        void read_images();
        void convert_pylon_to_ros(const CGrabResultPtr& image_ptr);

};

#endif