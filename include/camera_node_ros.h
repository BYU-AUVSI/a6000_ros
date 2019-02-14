/***************************
    Copyright (c) 2018, len0rd
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************/
#pragma once
#include "camera_connector.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.hpp>

// Our services. These headers are
// auto-generated by catkin from the files
// in the the srv/ directory (unless from std_srvs)
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32.h>
#include "a6000_ros/ConfigGet.h"
#include "a6000_ros/ConfigSet.h"

using namespace std;

class GphotoCameraROS {
public:
    GphotoCameraROS();

    void run(); //never returns

private:

    // ROS service callback for camera configuration
    bool configListServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool configGetServiceCallback(a6000_ros::ConfigGet::Request &req, a6000_ros::ConfigGet::Response &res);
    bool configSetServiceCallback(a6000_ros::ConfigSet::Request &req, a6000_ros::ConfigSet::Response &res);

    ros::ServiceServer config_list_srv_;
    ros::ServiceServer config_get_srv_;
    ros::ServiceServer config_set_srv_;

    CameraConnector cam_;
    ros::NodeHandle nh_private_;
    ros::Publisher focal_length_pub_;
    image_transport::ImageTransport img_transport_;
    image_transport::Publisher image_pub_;

    // for the actual images
    char* img_data_;
    unsigned long img_size_;
    // keep track of focal_length.
    // since focal length is almost definitely going to stay constant
    // during flight, we dont need to worry too much about tying a 
    // measurement with each image. Instead, we only publish when there's
    // a change to it
    float cur_focal_len_;
};