#pragma once
#include "camera_connector.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.hpp>

using namespace std;

class GphotoCameraROS {
public:
    GphotoCameraROS();

    void run(); //never returns

private:
    ros::NodeHandle nh_private_;
    image_transport::ImageTransport img_transport_;
    image_transport::Publisher image_pub_;
};