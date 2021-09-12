#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <string>
#include <chrono>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_ros_msgs/SpatialDetectionArray.h>
#include <depthai_ros_msgs/SpatialDetection.h>

using namespace std;
using namespace std::chrono;
using namespace cv;

dai::rosBridge::ImageConverter inputConverter(true);
auto startTime = steady_clock::now();
auto white = Scalar(255, 255, 255);
auto green = Scalar(0, 255, 0);
auto red   = Scalar(0, 0, 255);

class NNDConverterNode {
  private:
    
    ros::Subscriber img_sub;
    ros::Subscriber detections;
    //ros::Subscriber depth_sub;

    int frameNum, counter;
    float fps;
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    
    string nn_model, id_str, device_name;
    
    const vector<string> label_map_mobilenet = {
        "background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",   
        "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse", 
        "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};
    
    const vector<string> label_map_yolov4 = {
        "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
        "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
        "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
        "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
        "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
        "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
        "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
        "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
        "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
        "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
        "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
        "teddy bear",     "hair drier", "toothbrush"};

    depthai_ros_msgs::SpatialDetectionArray detectionArray;

    image_transport::ImageTransport it;
    image_transport::Publisher bb_pub; //image with bounding boxes

    void imageCallback(const sensor_msgs::ImagePtr& msg);
    void detectionCallback(const depthai_ros_msgs::SpatialDetectionArray& detectionMsg);

  public:
    NNDConverterNode();
};

void NNDConverterNode::imageCallback(const sensor_msgs::ImagePtr& msg)
{    
    Mat bbImage = inputConverter.rosMsgtoCvMat(*msg);
    
    int rows = bbImage.rows;
    int cols = bbImage.cols;

    Size s = bbImage.size();
    rows = s.height;
    cols = s.width;
    
    std_msgs::Header header; // empty header
    header.seq = frameNum; // user defined counter
    header.frame_id = device_name + "_rgb_camera_optical_frame";
    header.stamp = ros::Time::now(); // time
    
    if(true)
    {
        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }
        
        ROS_DEBUG("Number of detections: %ld", detectionArray.detections.size());
        for(int i=0; i<detectionArray.detections.size(); i++)
        {
            long int id = (long int)detectionArray.detections[i].results[0].id;

            if(nn_model == "mobilenet"){ id_str = label_map_mobilenet[id]; }
            else if(nn_model == "yolov4"){ id_str = label_map_yolov4[id]; }

            float confidence = detectionArray.detections[i].results[0].score * 100;
            ROS_INFO("Detection[%d]: %s-%d (%f\%)", i, id_str.c_str(), id, confidence);
            
            float m = (float)(rows/300.0);
            int offset = (int)((cols - 300*m)/2);
            int x1 = (int)((detectionArray.detections[i].bbox.center.x - detectionArray.detections[i].bbox.size_x/2.0)*m) + offset;
            int y1 = (int)((detectionArray.detections[i].bbox.center.y - detectionArray.detections[i].bbox.size_y/2.0)*m);
            int x2 = (int)((detectionArray.detections[i].bbox.center.x + detectionArray.detections[i].bbox.size_x/2.0)*m) + offset;
            int y2 = (int)((detectionArray.detections[i].bbox.center.y + detectionArray.detections[i].bbox.size_y/2.0)*m);

            putText(bbImage, id_str, Point(x1 + 10, y1 + 40), FONT_HERSHEY_TRIPLEX, 1.5, green, 2);
            stringstream confStr;
            confStr << fixed << setprecision(2) << confidence << " %";
            putText(bbImage, confStr.str(), Point(x1 + 10, y1 + 80), FONT_HERSHEY_TRIPLEX, 1.5, green, 2);

            stringstream depthX;
            depthX << "X: " << (int)(detectionArray.detections[i].position.x * 1000) << " mm";
            putText(bbImage, depthX.str(), Point(x1 + 10, y1 + 120), FONT_HERSHEY_TRIPLEX, 1.5, red, 2);
            
            stringstream depthY;
            depthY << "Y: " << (int)(detectionArray.detections[i].position.y * 1000) << " mm";
            putText(bbImage, depthY.str(), Point(x1 + 10, y1 + 160), FONT_HERSHEY_TRIPLEX, 1.5, red, 2);
            
            stringstream depthZ;
            depthZ << "Z: " << (int)(detectionArray.detections[i].position.z * 1000) << " mm";
            putText(bbImage, depthZ.str(), Point(x1 + 10, y1 + 200), FONT_HERSHEY_TRIPLEX, 1.5, red, 2);

            rectangle(bbImage, Rect(Point(x1, y1), Point(x2, y2)), white, 3);
        }
        
        stringstream fpsStr;
        fpsStr << "FPS: " << fixed << setprecision(2) << fps;
        putText(bbImage, fpsStr.str(), Point(10, rows - 20), FONT_HERSHEY_TRIPLEX, 1.5, white, 2);

        cv_bridge::CvImage bb_bridge;
        sensor_msgs::Image bb_msg;
        
        bb_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, bbImage);
        bbImage.release();
        bb_bridge.toImageMsg(bb_msg); // from cv_bridge to sensor_msgs::Image
        bb_pub.publish(bb_msg);
    }
    frameNum++;
}

void NNDConverterNode::detectionCallback(const depthai_ros_msgs::SpatialDetectionArray& detectionMsg)
{
    detectionArray = detectionMsg;
}

NNDConverterNode::NNDConverterNode() : nh(), pnh("~"), it(nh)
{
    frameNum = 0;
    counter = 0;
    fps = 0;
    
    int bad_params = 0;
    
    bad_params += !pnh.getParam("nn_model", nn_model);
    bad_params += !pnh.getParam("device_name", device_name);
            
    if (bad_params > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }
    
    pnh.param<string>("nn_model", nn_model, "");
    pnh.param<string>("device_name", device_name, "");

    bb_pub = it.advertise("bb_out", 30);
    img_sub      = nh.subscribe("video_in", 1, &NNDConverterNode::imageCallback, this);
    detections   = nh.subscribe("detections", 1, &NNDConverterNode::detectionCallback, this);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "rgbd_converter");

    NNDConverterNode* vc_node = new NNDConverterNode();

    ros::spin();

    return 0;
}
