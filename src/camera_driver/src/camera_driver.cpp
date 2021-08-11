#include "opencv2/opencv.hpp"
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv){
    ros::init(argc, argv, "simple_publication_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/raw", 1);
    ros::Rate rate = ros::Rate(36);
    ROS_INFO_STREAM("publication node started");

    VideoCapture cap("/home/cy/apps/tokyo_road.mp4"); 
    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    size_t idx = 0;
    while(1){

        Mat frame;
        cap >> frame;
    
        if (frame.empty())
            break;
            
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = ros::Time::now();
        pub.publish(msg);
        rate.sleep();
        // imwrite("Frame"+to_string(++idx)+".jpg", frame);
    }
    
    cap.release();
    return 0;
}