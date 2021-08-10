#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "json.hpp"

#include "class_timer.hpp"
#include "class_detector.h"

using namespace std;
using namespace cv;
using json = nlohmann::json;

mutex mtx; 
bool worker_stop = false;
thread worker_thread;
vector<sensor_msgs::ImageConstPtr> msg_queue;
vector<BatchResult> batch_res;

Config init_model_cfg(string cfg_file, string weights_file, string calib_file) {
    Config config_v5;
	config_v5.net_type = YOLOV5;
	config_v5.detect_thresh = 0.5;
	config_v5.file_model_cfg = cfg_file;
	config_v5.file_model_weights = weights_file;
	config_v5.calibration_image_list_file_txt = calib_file;
	config_v5.inference_precison = FP32;
    return config_v5;
}

Detector* init_detector(Config cfg) {
    Detector* detector(new Detector());
    detector->init(cfg);
    return detector;
}

json format_res(vector<BatchResult> batch_res) {
    json objs = json::array();
    for (const auto &r : batch_res[0])
    {
        json j;
        j["id"] = r.id;
        j["score"] = r.prob;
        j["rect"] = {r.rect.x, r.rect.y};
        objs.push_back(j);
        // std::cout <<"batch "<<i<< " id:" << r.id << " prob:" << r.prob << " rect:" << r.rect << std::endl;
        // cv::rectangle(batch_img[i], r.rect, cv::Scalar(255, 0, 0), 2);
        // std::stringstream stream;
        // stream << std::fixed << std::setprecision(2) << "id:" << r.id << "  score:" << r.prob;
        // cv::putText(batch_img[i], stream.str(), cv::Point(r.rect.x, r.rect.y - 5), 0, 0.5, cv::Scalar(0, 0, 255), 2);
    }
    return objs;
}

void worker(Detector* detector, ros::Publisher* pub){
  ROS_INFO("Worker start.");
  while(!worker_stop){
    mtx.lock();
    if (msg_queue.size() == 0){
      mtx.unlock();
      std::this_thread::sleep_for(0.1*std::chrono::seconds(1));
      continue;
    }
    auto msg = msg_queue.back();
    msg_queue.pop_back();
    mtx.unlock();

    try{
      auto img = cv_bridge::toCvShare(msg, "bgr8")->image;
      detector->detect({img}, batch_res);
      auto res_json = format_res(batch_res);
      json final_res;
      final_res["timestamp"] = (msg->header).stamp.toSec();
      final_res["results"] = res_json;
      std_msgs::String smsg;
      smsg.data = final_res.dump();
    //   smsg.header.stamp = msg->header->stamp;
      pub->publish(msg);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("Could not convert from '%s', got error :%s", \
        msg->encoding.c_str(), e.what());
    }
    catch (...){
      ROS_ERROR("predict fail.");
    }
  }
  ROS_INFO("Worker exit.");
}

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
    mtx.lock();
    if (msg_queue.size() > 1) {
        msg_queue.pop_back();
    }
    msg_queue.emplace_back(msg);
    mtx.unlock();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "perception_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/raw", 1, image_callback);

    auto cfg = init_model_cfg("./configs/yolov5-5.0/yolov5s6.cfg", "./configs/yolov5-5.0/yolov5s6.weights", "./configs/calibration_images.txt");
    auto detector = init_detector(cfg);

    ros::NodeHandle n;
    ros::Publisher res_pub = n.advertise<std_msgs::String>("/detection", 1);
    auto worker_thread = std::thread(worker, detector, &res_pub);

    ROS_INFO_STREAM("subscriber started");
    ros::spin();
    worker_stop = true;
    worker_thread.join();

    return 0;
}