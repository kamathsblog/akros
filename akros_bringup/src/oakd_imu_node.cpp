#include <ros/ros.h>

#include <iostream>
#include <cstdio>
#include <sensor_msgs/Imu.h>
#include <functional>

// Inludes common necessary includes for development using depthai library
#include <depthai/depthai.hpp>
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImuConverter.hpp>


dai::Pipeline createPipeline(){
    dai::Pipeline pipeline;

    auto imu                  = pipeline.create<dai::node::IMU>();
    auto xoutImu              = pipeline.create<dai::node::XLinkOut>();

    xoutImu->setStreamName("imu");
    
    //Imu
    imu->enableIMUSensor({dai::IMUSensor::ROTATION_VECTOR, dai::IMUSensor::LINEAR_ACCELERATION, dai::IMUSensor::GYROSCOPE_CALIBRATED}, 400);
    imu->setMaxBatchReports(1); // Get one message only for now.
    
    imu->out.link(xoutImu->input);

    return pipeline;
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle pnh("~");
    
    std::string deviceName;

    int badParams = 0;
    badParams += !pnh.getParam("camera_name", deviceName);

    if (badParams > 0){   
        std::cout << " Bad parameters -> " << badParams << std::endl;
        throw std::runtime_error("Couldn't find some of the parameters");
    }

    dai::Pipeline pipeline = createPipeline();

    dai::Device device(pipeline);

    auto imuQueue = device.getOutputQueue("imu", 30, false);

    dai::rosBridge::ImuConverter imuConverter(deviceName +"_imu_frame");

    dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData> ImuPublish(imuQueue, pnh, std::string("imu"),
                                                                               std::bind(&dai::rosBridge::ImuConverter::toRosMsg, 
                                                                               &imuConverter, std::placeholders::_1, std::placeholders::_2), 
                                                                               30, "", "imu");
    ImuPublish.addPubisherCallback();
    
    ros::spin();
    return 0;
}