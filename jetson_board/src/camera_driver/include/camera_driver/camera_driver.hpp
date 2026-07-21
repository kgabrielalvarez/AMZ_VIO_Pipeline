// Code based on:
// 1. https://ja.docs.baslerweb.com/pylonapi/cpp/sample_code#utility_grabvideo
// Intended hardware:
// 1. Camera model: https://www.baslerweb.com/en/shop/a2a1920-168mgc/
// 2. GMSL Adapter board: https://www.baslerweb.com/en/shop/gmsl-adapter-kit-for-orin-nano-dev-kit/

#ifndef camera_driver_h
#define camera_driver_h

// Include Pylon specific libraries
#include <pylon/PylonIncludes.h>

// Include ROS2 libraries
#include "rclcpp/rclcpp.hpp"

// Include namespaces
using namespace Pylon;
using namespace GenApi; // From the GenICam standard

// Driver to connect camera to Jetson over GMSL adpater board
class camera_driver : public rclcpp::Node {

    public:

        // Constructor
        camera_driver();

    private:
        // TO-DO

};

#endif