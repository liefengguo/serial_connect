#ifndef VRFKREADER_H
#define VRFKREADER_H

#include "ros/ros.h"
#include <serial_connect/VRTK.h>

#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "geometry_msgs/Vector3.h"
#include <std_msgs/Int32.h>
using namespace serial_connect;
class VRFKReader {
private:
    ros::NodeHandle nh;
    ros::Subscriber subscriber;
    VRTK vrTkData;

public:
    VRFKReader();
    void vrTkCallback(const VRTK::ConstPtr& msg);
    std_msgs::Header getHeader() const;
    std::string getPoseFrame() const;
    std::string getKinFrame() const;
    geometry_msgs::PoseWithCovariance getPose() const;
    geometry_msgs::TwistWithCovariance getVelocity() const;
    geometry_msgs::Vector3 getAcceleration() const;
    int16_t getFusionStatus() const;
    int16_t getIMUBiasStatus() const;
    int16_t getGNSSStatus() const;
    int16_t getWheelspeedStatus() const;
    std::string getVersion() const;
};

#endif  // VRFKREADER_H
