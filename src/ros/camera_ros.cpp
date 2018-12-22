#include "camera_node_ros.h"

GphotoCameraROS::GphotoCameraROS() : nh_private_("~"), img_transport_(nh_private_) {

    image_pub_ = img_transport_.advertise("img", 1);

    config_list_srv_ = nh_private_.advertiseService("config_list", &GphotoCameraROS::configListServiceCallback, this);
    config_get_srv_  = nh_private_.advertiseService("config_get", &GphotoCameraROS::configGetServiceCallback, this);

    // setup camera. but dont connect to it yet
    cam_ = CameraConnector();

}

void GphotoCameraROS::run() {

    // continuously try to connect to the camera until it works
    cam_.blockingConnect();

    char* imgData;
    unsigned long imgSize;

    while (nh_private_.ok()) { // while ROS is up, and this node hasn't been told to close
        if (cam_.captureImage((const char**) &imgData, &imgSize)) {

            // so the raw data we get from gphoto2 / the captureImage function
            // is straight up jpg data. not raw image bytes or anything. so in order to deal
            // with that we need to first put it into a Mat and then call imdecode
            // to properly bundle it into a sensor_msg
            // CV_8SC3 is opencv talk for 8 bit signed 3 channel image data
            cv::Mat rawData(1, imgSize, CV_8SC3, (void*) imgData);

            cv_bridge::CvImage cvbImg;
            cvbImg.image = cv::imdecode(rawData, -CV_LOAD_IMAGE_COLOR);
            cvbImg.encoding = "bgr8";

            sensor_msgs::Image ros_image;
            cvbImg.toImageMsg(ros_image);
            image_pub_.publish(ros_image);

            // spinning here allows us to respond to any pending callbacks 
            // (generally from service calls waiting to be fulfilled)
            ros::spinOnce(); 

        } else {
            // Need more failure handling here
            cout << "cap failed" << endl;
            // only do error handling if the node is supposed to continue existing
            // if (nh_private_.ok()) {}
        }
    }

    // make sure to properly detach camera resources!
    cam_.close();
}

bool GphotoCameraROS::configListServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
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
        }
        
    }
    return true;
}

bool GphotoCameraROS::configGetServiceCallback(a6000_ros::ConfigGet::Request &req, a6000_ros::ConfigGet::Response &res) {
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
            // the setting exists lets get info on it
            res.exists = true;

            char currentValue[50];
            if (cam_.getConfigStringValue(setting, (char*) currentValue)) {
                std::string currentStr(currentValue);
                res.currentValue = currentStr;
            } else {
                res.currentValue = "ERR:: Something went wrong trying to get the current value";
            }

            std::string configOpts = cam_.getConfigOptionsString(setting);
            if (!configOpts.empty()) {
                res.possibleValues = configOpts;
            } else {
                res.possibleValues = "ERR:: Something went wrong trying to get the possible values";
            }
        }
    }
    return true;
}