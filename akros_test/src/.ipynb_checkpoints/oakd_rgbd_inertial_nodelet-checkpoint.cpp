#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <iostream>
#include <cstdio>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <camera_info_manager/camera_info_manager.h>
#include <functional>

// Inludes common necessary includes for development using depthai library
#include <depthai/depthai.hpp>
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImuConverter.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/DisparityConverter.hpp>

namespace akros_bringup{

 class RGBDInertialNodelet : public nodelet::Nodelet
{
    std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData>> ImuPublish;
    std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> rgbPublish, leftPublish, rightPublish, depthPublish;
    std::unique_ptr<dai::rosBridge::BridgePublisher<stereo_msgs::DisparityImage, dai::ImgFrame>> dispPublish;
    
    std::unique_ptr<dai::rosBridge::ImuConverter> imuConverter;
    std::unique_ptr<dai::rosBridge::ImageConverter> leftConverter, rightConverter, rgbConverter;
    std::unique_ptr<dai::rosBridge::DisparityConverter> dispConverter;
    std::unique_ptr<dai::Device> _dev;

    public:
        virtual void onInit() override {

            auto& pnh = getPrivateNodeHandle();
            
            std::string deviceName, mode;
            int badParams = 0; 
            int stereo_fps = 0;
            int confidence = 200;
            int LRcheckthresh = 5;
            bool lrcheck, extended, subpixel, enableRGBD, rectify, depth_aligned;

            badParams += !pnh.getParam("camera_name", deviceName);
            badParams += !pnh.getParam("mode", mode);
            badParams += !pnh.getParam("lrcheck",  lrcheck);
            badParams += !pnh.getParam("extended",  extended);
            badParams += !pnh.getParam("subpixel",  subpixel);
            badParams += !pnh.getParam("rectify",  rectify);
            badParams += !pnh.getParam("depth_aligned",  depth_aligned);
            badParams += !pnh.getParam("stereo_fps",  stereo_fps);
            badParams += !pnh.getParam("confidence", confidence);
            badParams += !pnh.getParam("LRcheckthresh", LRcheckthresh);
            
            if (badParams > 0)
            {   
                std::cout << " Bad parameters -> " << badParams << std::endl;
                throw std::runtime_error("Couldn't find some of the parameters");
            }

            if(mode == "rgbd"){ enableRGBD = true; }
            else{ enableRGBD = false; }

            dai::Pipeline pipeline = createPipeline(enableRGBD, lrcheck, extended, subpixel, rectify, depth_aligned, stereo_fps, confidence, LRcheckthresh);
            
            _dev = std::make_unique<dai::Device>(pipeline);

            //QUEUES
            std::shared_ptr<dai::DataOutputQueue> stereoQueue;
            if (enableRGBD) {
                stereoQueue = _dev->getOutputQueue("depth", 30, false);
            }else{
                stereoQueue = _dev->getOutputQueue("disparity", 30, false); }
            auto imuQueue = _dev->getOutputQueue("imu", 30, false);
            auto imgQueue = _dev->getOutputQueue("rgb", 30, false);
            auto leftQueue = _dev->getOutputQueue("left", 30, false);
            auto rightQueue = _dev->getOutputQueue("right", 30, false);
            
            auto calibrationHandler = _dev->readCalibration();

            //CONVERTERS
            imuConverter = std::make_unique<dai::rosBridge::ImuConverter>(deviceName + "_imu_frame");
            leftConverter = std::make_unique<dai::rosBridge::ImageConverter>(deviceName + "_left_camera_optical_frame", true);
            rightConverter = std::make_unique<dai::rosBridge::ImageConverter>(deviceName + "_right_camera_optical_frame", true);
            rgbConverter = std::make_unique<dai::rosBridge::ImageConverter>(deviceName + "_rgb_camera_optical_frame", true);
            auto depthconverter = std::make_unique<dai::rosBridge::ImageConverter>(deviceName + "_right_camera_optical_frame", true);
            if(depth_aligned){
                depthconverter = std::make_unique<dai::rosBridge::ImageConverter>(deviceName + "_rgb_camera_optical_frame", true);
            }
            std::string tfSuffix = depth_aligned ? "_rgb_camera_optical_frame" : "_right_camera_optical_frame";
            dispConverter = std::make_unique<dai::rosBridge::DisparityConverter>(deviceName + tfSuffix , 880, 7.5, 20, 2000);
            
            //CAMERA INFO
            auto leftCameraInfo = leftConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, 1280, 720); 
            auto rightCameraInfo = rightConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, 1280, 720); 
            auto rgbCameraInfo = rgbConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, 1280, 720); 
            auto depthCameraInfo = depth_aligned ? rgbConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, 1280, 720) : rightCameraInfo;
            auto disparityCameraInfo = depth_aligned ? rgbConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, 1280, 720) : rightCameraInfo;
            
            const std::string leftTopicName = rectify ? std::string("left/image_rect") : std::string("left/image_raw");
            const std::string rightTopicName = rectify ? std::string("right/image_rect") : std::string("right/image_raw");
            
            //PUBLISH IMU
            ImuPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData>>(imuQueue, pnh, std::string("imu"),
                                                                               std::bind(&dai::rosBridge::ImuConverter::toRosMsg, 
                                                                               imuConverter.get(), std::placeholders::_1, std::placeholders::_2), 
                                                                               30, "", "imu");
            ImuPublish->addPubisherCallback();
            
            //PUBLISH COLOR
            rgbPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(imgQueue, pnh, std::string("color/image"),
                                                                            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                            rgbConverter.get(), std::placeholders::_1, std::placeholders::_2),
                                                                            30, rgbCameraInfo, "color");
            rgbPublish->addPubisherCallback();
            
            if(enableRGBD){
                //PUBLISH DEPTH
                depthPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(stereoQueue, pnh,
                                                                                     std::string("stereo/depth"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     depthconverter.get(), std::placeholders::_1, std::placeholders::_2), 
                                                                                     30, depthCameraInfo, "stereo");
                depthPublish->addPubisherCallback();
            }
            else{
                //PUBLISH DISPARITY
                dispPublish = std::make_unique<dai::rosBridge::BridgePublisher<stereo_msgs::DisparityImage, dai::ImgFrame>>(stereoQueue, pnh, 
                                                                                     std::string("stereo/disparity"),
                                                                                     std::bind(&dai::rosBridge::DisparityConverter::toRosMsg, 
                                                                                     dispConverter.get(), std::placeholders::_1, std::placeholders::_2), 
                                                                                     30, disparityCameraInfo, "stereo");
                dispPublish->addPubisherCallback();
                
                //PUBLISH LEFT
                leftPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(leftQueue, pnh, leftTopicName,
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                    leftConverter.get(), std::placeholders::_1, std::placeholders::_2), 
                                                                                    30, leftCameraInfo, "left");
                leftPublish->addPubisherCallback();
        
                //PUBLISH RIGHT
                rightPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(rightQueue, pnh, rightTopicName,
                                                                                  std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                  rightConverter.get(), std::placeholders::_1, std::placeholders::_2),
                                                                                  30, rightCameraInfo, "right");  
                rightPublish->addPubisherCallback();
            }
        }


    dai::Pipeline createPipeline(bool enableRGBD, bool lrcheck, bool extended, bool subpixel, bool rectify, bool depth_aligned, int stereo_fps, int confidence, int LRcheckthresh){
        dai::Pipeline pipeline;

        auto imu                  = pipeline.create<dai::node::IMU>();
        auto xoutImu              = pipeline.create<dai::node::XLinkOut>();
        auto stereo               = pipeline.create<dai::node::StereoDepth>();
        auto xoutDepth            = pipeline.create<dai::node::XLinkOut>();
        auto camRgb               = pipeline.create<dai::node::ColorCamera>();
        auto xoutRgb              = pipeline.create<dai::node::XLinkOut>();
        auto monoLeft             = pipeline.create<dai::node::MonoCamera>();
        auto xoutLeft             = pipeline.create<dai::node::XLinkOut>();
        auto monoRight            = pipeline.create<dai::node::MonoCamera>();
        auto xoutRight            = pipeline.create<dai::node::XLinkOut>();

        xoutImu->setStreamName("imu");
        if(enableRGBD){ xoutDepth->setStreamName("depth"); }
        else { xoutDepth->setStreamName("disparity"); }
        xoutRgb->setStreamName("rgb");
        xoutLeft->setStreamName("left");
        xoutRight->setStreamName("right");
        
        //Imu
        imu->enableIMUSensor({dai::IMUSensor::ROTATION_VECTOR, dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 400);
        imu->setMaxBatchReports(1); // Get one message only for now.
        
        // StereoDepth
        stereo->initialConfig.setConfidenceThreshold(confidence); //Known to be best
        stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
        stereo->initialConfig.setLeftRightCheckThreshold(LRcheckthresh); //Known to be best
        stereo->setLeftRightCheck(lrcheck);
        stereo->setExtendedDisparity(extended);
        stereo->setSubpixel(subpixel);
        if(enableRGBD && depth_aligned) stereo->setDepthAlign(dai::CameraBoardSocket::RGB);

        //RGBCamera
        camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
        camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        // the ColorCamera is downscaled from 1080p to 720p. Otherwise, the aligned depth is automatically upscaled to 1080p
        camRgb->setIspScale(2, 3);
        // For now, RGB needs fixed focus to properly align with depth. This value was used during calibration
        camRgb->initialControl.setManualFocus(135);
        
        // MonoCamera
        monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        monoLeft->setFps(stereo_fps);
        monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
        monoRight->setFps(stereo_fps);
        
        imu->out.link(xoutImu->input);
        
        if(enableRGBD){ stereo->depth.link(xoutDepth->input); }
        else{ stereo->disparity.link(xoutDepth->input); }
        
        camRgb->isp.link(xoutRgb->input);

        if(rectify){
            stereo->rectifiedLeft.link(xoutLeft->input);
            stereo->rectifiedRight.link(xoutRight->input);     
        }else{
            stereo->syncedLeft.link(xoutLeft->input);
            stereo->syncedRight.link(xoutRight->input);
        }
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        return pipeline;
    }
};

PLUGINLIB_EXPORT_CLASS(akros_bringup::RGBDInertialNodelet, nodelet::Nodelet)
}   // namespace oakd
