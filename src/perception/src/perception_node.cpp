#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

int idx = 0;

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        imwrite("sub-"+to_string(++idx)+".jpg", image);
        ROS_INFO("sub success");
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "perception_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/raw", 1, image_callback);
    ROS_INFO_STREAM("subscriber started");

    ros::spin();
}