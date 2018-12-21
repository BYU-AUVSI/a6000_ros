#include "camera_node_ros.h"

GphotoCameraROS::GphotoCameraROS() : nh_private_("~"), img_transport_(nh_private_) {

    image_pub_ = img_transport_.advertise("img", 1);
    cout << "setup publisher" << endl;

}

void GphotoCameraROS::run() {
    cout << "setup cam" << endl;
    CameraConnector cam = CameraConnector();
    cam.blockingConnect();

    char* imgData;
    unsigned long imgSize;

    while (nh_private_.ok()) {
        if (cam.captureImage((const char**) &imgData, &imgSize)) {

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
            cout << "done" << endl;

        } else {
            cout << "cap failed" << endl;
        }
    }
}
