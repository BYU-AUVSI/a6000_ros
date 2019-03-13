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
#include "camera_node_ros.h"

GphotoCameraROS::GphotoCameraROS() : nh_private_("~"), img_transport_(nh_private_) {

    // private nh is relative to our node ie: a6000_ros_node is prepended to this:
    // this disables depth and theora plugins which are unnecessary for our stuff.
    //      basically just want to clean up the rostopic list of all unnecessary crap. 
    //      also dont worry about compressed images
    nh_private_.setParam("/img/disable_pub_plugins", "['image_transport/compressedDepth', 'image_transport/theora', 'image_transport/compressed']");

    image_pub_ = img_transport_.advertise("img", 1);

    focal_length_pub_ = nh_private_.advertise<std_msgs::Float32>("img/focal_length", 1);
    cur_focal_len_ = -1; //initialize to garbage value so it gets updated

    // setup our configuration services
    config_list_srv_ = nh_private_.advertiseService("config_list", &GphotoCameraROS::configListServiceCallback, this);
    config_get_srv_  = nh_private_.advertiseService("config_get",  &GphotoCameraROS::configGetServiceCallback, this);
    config_set_srv_  = nh_private_.advertiseService("config_set",  &GphotoCameraROS::configSetServiceCallback, this);

    // setup camera. but dont connect to it yet
    cam_ = CameraConnector();
}

void GphotoCameraROS::run() {

    // continuously try to connect to the camera until it works
    cam_.blockingConnect();

    std_msgs::Float32Ptr focalLengthMsg(new std_msgs::Float32);
    float measuredFocalLength = -1;

    while (nh_private_.ok()) { // while ROS is up, and this node hasn't been told to close
        if (!cam_.isConnected()) {
            cam_.attemptConnection();
        } else if (cam_.captureImage((const char**) &img_data_, &img_size_)) {
            ros::Time timestamp = ros::Time::now(); // get ts as close to capture as possible

            if (cam_.lastImageHasEXIF()) {
                measuredFocalLength = cam_.getExif().FocalLength;
                // this is mildly inelegant, but since the focal length 
                // shouldnt really change ever, we dont need to worry too
                // much about it syncing up with images 
                // if you do need to worry about that for some reason (fl syncing per img),
                // then roll a custom message with image and focal length
                // or create a message with a std_msgs/Header and a float to publish fl
                focalLengthMsg->data = measuredFocalLength;
                focal_length_pub_.publish(focalLengthMsg);

                if (measuredFocalLength != cur_focal_len_) {
                    cur_focal_len_ = measuredFocalLength;
                    printf("Lens focal length    : %f mm\n", measuredFocalLength);
                }
            }

            // so the raw data we get from gphoto2 / the captureImage function
            // is straight up jpg data. not raw image bytes or anything. so in order to deal
            // with that we need to first put it into a Mat and then call imdecode
            // to properly bundle it into a sensor_msg
            // CV_8SC3 is opencv talk for 8 bit signed 3 channel image data
            // cv::Mat rawData(1, img_size_, CV_8SC3, (void*) img_data_);

            cv_bridge::CvImage cvbImg;
            // cvbImg.image = cv::imdecode(rawData, -CV_LOAD_IMAGE_COLOR);
            // looking to optimize this part as much as possible:
            cvbImg.image = cv::imdecode(cv::Mat(1, img_size_, CV_8SC3, (void*) img_data_), CV_LOAD_IMAGE_UNCHANGED);
            cvbImg.encoding = "bgr8"; // bgr is opencv default

            sensor_msgs::ImagePtr ros_image = cvbImg.toImageMsg();
            ros_image->header.stamp = timestamp;

            image_pub_.publish(ros_image);

        } else {
            // Need more failure handling here
            ROS_WARN("Camera capture failed, attempting to reconnect");
            // only do error handling if the node is supposed to continue existing
            if (nh_private_.ok()) {
                // for now our error handling just involves attempting to
                // reconnect the camera
                cam_.attemptConnection();
            }
        }
        // spinning here allows us to respond to any pending callbacks 
        // (generally from service calls waiting to be fulfilled)
        ros::spinOnce(); 
    }

    // make sure to properly detach camera resources!
    cam_.close();
}

bool GphotoCameraROS::configListServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    // if we get a weird error, we should just reset the camera 
    // driver connection by closing and opening the camera again
    bool needsReset = false; 

    if (!cam_.isConnected()) {
        res.success = false;
        res.message = "Camera is not currently connected!";
    } else {
        std::string msg = "";
        try {
            msg = cam_.getAllConfigBasicInfo(A6000Config::ALL);
            if (msg.empty()) {
                // For some reason an empty string was returned without
                // any type of exception. This could happen if the input vector to the
                // above function was empty, but I'm not sure why else it would happen
                res.success = false;
                res.message = "Got nothing when attempting to get configuration info";
                needsReset = true;
            } else {
                // Success!
                res.success = true;
                res.message = msg;
            }
        } catch (std::exception &e) {
            // Who knows what happened, but something failed
            ROS_WARN("Something went wrong while trying to get config info:: %s", e.what());
            res.success = false;
            res.message = e.what();
            needsReset = true;
        }   
    }

    if (needsReset) {
        cam_.close();
    }

    return true;
}

bool GphotoCameraROS::configGetServiceCallback(a6000_ros::ConfigGet::Request &req, a6000_ros::ConfigGet::Response &res) {
    // if we get a weird error, we should just reset the camera 
    // driver connection by closing and opening the camera again
    bool needsReset = false; 
    
    if (!cam_.isConnected()) {
        res.exists = false;
        res.currentValue = "Camera is not currently connected!";
    } else {
        
        //put the setting name into upper case, since that's how we've made config names work
        std::string configName = req.configName;
        std::transform(configName.begin(), configName.end(), configName.begin(), ::toupper);

        const ConfigSetting* setting = A6000Config::getConfigSetting(configName.c_str());

        if (setting == nullptr) {
            res.exists = false;
            res.currentValue = "Could not locate setting named: " + configName;
        } else {
            // the setting exists, lets get info on it
            res.exists = true;

            //get its current value first
            char currentValue[50];
            if (cam_.getConfigStringValue(setting, (char*) currentValue)) {
                std::string currentStr(currentValue);
                res.currentValue = currentStr;
            } else {
                res.currentValue = "ERR:: Something went wrong trying to get the current value";
                needsReset = true;
            }

            // get a description of all possible options for the setting
            std::string configOpts = cam_.getConfigOptionsString(setting);
            if (!configOpts.empty()) {
                res.possibleValues = configOpts;
            } else {
                res.possibleValues = "ERR:: Something went wrong trying to get the possible values";
            }
        }
    }

    if (needsReset) {
        cam_.close();
    }

    return true;
}

bool GphotoCameraROS::configSetServiceCallback(a6000_ros::ConfigSet::Request &req, a6000_ros::ConfigSet::Response &res) {
    // if we get a weird error, we should just reset the camera 
    // driver connection by closing and opening the camera again
    bool needsReset = false; 

    if (!cam_.isConnected()) {
        res.success = false;
        res.message = "Camera is not currently connected!";
    } else  {
        std::string configName = req.configName;
        std::string configValue = req.value;

        std::transform(configName.begin(), configName.end(), configName.begin(), ::toupper);
        // try and get the setting by the provided name
        const ConfigSetting* setting = A6000Config::getConfigSetting(configName.c_str());

        if (setting == nullptr) {
            res.success = false;
            res.message = "Could not locate setting named: " + configName;
        } else {
            // the setting exists, try and change its value
            // setConfigValue method will check that the value we're 
            // trying to set it to is valid
            // usleep(3000000);
            if (!cam_.setConfigValue(setting, configValue)) {
                res.success = false;
                res.message = "Failed to set value for " + configName + ". Is the value valid?";
            } else {
                // now get the settings current value to verfiy it was successfully changed
                char currentValue[50];
                if (!cam_.getConfigStringValue(setting, (char*) currentValue)) {
                    // for some reason we failed to get the current config value
                    res.success = false;
                    res.message = "Unable to verify value was set. Its possible this operation was successful";
                    needsReset = true;
                } else {
                    // make sure the settings value is what we told it to be:
                    std::string currentStr(currentValue);
                    if (currentStr.compare(configValue) != 0) {
                        res.success = false;
                        res.message = "Driver told us the value was set, but our verification failed. You should retry";
                        needsReset = true;
                    } else {
                        res.success = true;
                        res.message = "Successfully updated camera configuration!";
                    }
                }
            }
        }
    }

    if (needsReset) {
        cam_.close();
    }

    return true;
}