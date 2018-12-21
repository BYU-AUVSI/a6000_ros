#include "camera_node_ros.h"

GphotoCameraROS::GphotoCameraROS() : nh_private_("~"), img_transport_(nh_private_) {

    image_pub_ = img_transport_.advertise("img", 1);

    config_list_srv_ = nh_private_.advertiseService("config_list", &GphotoCameraROS::configListServiceCallback, this);

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
        }
    }

    // make sure to properly detach camera resources!
    cam_.close();
}


bool GphotoCameraROS::configListServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    cout << "were innnn" << endl;
    ROS_INFO("were in ros edition");
    res.message = "hey whats up helooooooooo";
    return true;
}
