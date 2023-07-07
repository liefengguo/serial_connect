#include "../include/serial_connect/six_decition_radar.h"

DistanceSensor::DistanceSensor() {
    sub_ = nh_.subscribe("a22_radar", 1, &DistanceSensor::distanceCallback, this);
    nh_.param<int>("six_decition_radar/targetDistance", targetDistance, 230);
    nh_.param<int>("six_decition_radar/distanceThreshold", distanceThreshold, 80);
    nh_.param<int>("six_decition_radar/bufferSize", bufferSize, 20);
    nh_.param<int>("six_decition_radar/threshold", threshold, 50);
}
void DistanceSensor::filterBigNum(int val,int lastVal){
    
}
void DistanceSensor::distanceCallback(const serial_connect::a22_data::ConstPtr& msg) {
    distance_ = msg->a22_datas;
    static AdaptiveFilter filter1(bufferSize,threshold); 
    static AdaptiveFilter filter2(bufferSize,threshold); 
    static AdaptiveFilter filter3(bufferSize,threshold); 
    static AdaptiveFilter filter4(bufferSize,threshold);
    static AdaptiveFilter filter5(bufferSize,threshold);
    static AdaptiveFilter filter6(bufferSize,threshold); 

    int lastDistance1,lastDistance2,lastDistance3,lastDistance4,lastDistance5,lastDistance6;

    filteredDistance0 = filter1.filter(distance_[0]);
    filteredDistance1 = filter2.filter(distance_[1]);
    filteredDistance2 = filter3.filter(distance_[2]);
    filteredDistance3 = filter4.filter(distance_[3]);  
    filteredDistance4 = filter5.filter(distance_[4]);
    filteredDistance5 = filter6.filter(distance_[5]);

    std::cout<<"真值1："<<distance_[1]<< "距离："<<filteredDistance1<<std::endl;
    std::cout<<"真值2："<<distance_[2]<< "距离："<<filteredDistance2<<std::endl;
    std::cout<<"真值0："<<distance_[0]<< "距离："<<filteredDistance0<<std::endl;

    // 判断距离是否偏离目标距离范围
    // if (filteredDistance1 < targetDistance - distanceThreshold ) {
    //     // 距离过近，需要向左调整车辆行驶方向
    //     // 在这里添加调整车辆方向的代码
    //     std::cout<< "距离过近，left:"<<filteredDistance2 - targetDistance<<std::endl;
    // } else if (filteredDistance1 > targetDistance + distanceThreshold) {
    //     // 距离过远，需要向左调整车辆行驶方向
    //     // 在这里添加调整车辆方向的代码
    //     std::cout<< "distance too far， right please!!"<<filteredDistance2 - targetDistance<<std::endl;
    // } else {
    //     // 距离在目标范围内，维持当前行驶方向
    //     // 在这里添加维持当前行驶方向的代码
    //     std::cout<< "OK! go "<<std::endl;
    // }
}

int DistanceSensor::getFilteredDistance1() const {
    return filteredDistance1;
}

int DistanceSensor::getFilteredDistance2() const {
    return filteredDistance2;
}

int DistanceSensor::getFilteredDistance3() const {
    return filteredDistance3;
}

int DistanceSensor::getFilteredDistance4() const {
    return filteredDistance4;
}

int DistanceSensor::getFilteredDistance5() const {
    return filteredDistance5;
}

int DistanceSensor::getFilteredDistance0() const {
    return filteredDistance0;
}


// int main(int argc, char** argv) {
//     ros::init(argc, argv, "distance_monitor");
//     ros::NodeHandle nh;

//     DistanceSensor sensor;
//     ros::spin();

//     return 0;
// }
