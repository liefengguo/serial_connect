#ifndef DECITION_RADAR_H
#define DECITION_RADAR_H

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include "adaptive_filter.h"
#include <std_msgs/Int32.h>
#include <serial_connect/a22_data.h>

class DistanceSensor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::vector<int32_t> distance_;

    int targetDistance ;  // 目标距离
    int distanceThreshold ;  // 距离阈值
    int bufferSize; 
    int threshold;
    int filteredDistance1 ,filteredDistance2,filteredDistance3,filteredDistance4,filteredDistance5,filteredDistance0;
public:
    DistanceSensor();
    void filterBigNum(int val,int lastVal);
    void distanceCallback(const serial_connect::a22_data::ConstPtr& msg);

    int getFilteredDistance1() const;
    int getFilteredDistance2() const;
    int getFilteredDistance3() const;
    int getFilteredDistance4() const;
    int getFilteredDistance5() const;
    int getFilteredDistance0() const;

};
#endif  // ADAPTIVE_FILTER_H
