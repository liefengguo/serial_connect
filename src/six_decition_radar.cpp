#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include "../include/serial_connect/adaptive_filter.h"
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
public:
    DistanceSensor() {
        sub_ = nh_.subscribe("a22_radar", 1, &DistanceSensor::distanceCallback, this);

        nh_.param<int>("six_decition_radar/targetDistance", targetDistance, 230);
        nh_.param<int>("six_decition_radar/distanceThreshold", distanceThreshold, 80);
        nh_.param<int>("six_decition_radar/bufferSize", bufferSize, 20);
        nh_.param<int>("six_decition_radar/threshold", threshold, 50);
    }
    void filterBigNum(int val,int lastVal){
        
    }
    void distanceCallback(const serial_connect::a22_data::ConstPtr& msg) {
        distance_ = msg->a22_datas;
        static AdaptiveFilter filter(bufferSize,threshold); 
        int filteredDistance1 ,filteredDistance2,filteredDistance3,filteredDistance4,filteredDistance5,filteredDistance0;
        int lastDistance1,lastDistance2,lastDistance3,lastDistance4,lastDistance5,lastDistance6;
        
        filteredDistance0 = filter.filter(distance_[0]); 
        filteredDistance1 = filter.filter(distance_[1]); 
        filteredDistance2 = filter.filter(distance_[2]);  
        filteredDistance3 = filter.filter(distance_[3]);  
        filteredDistance4 = filter.filter(distance_[4]);
        filteredDistance5 = filter.filter(distance_[5]);

        std::cout<<"真值1："<<distance_[1]<< "距离："<<filteredDistance1<<std::endl;
        std::cout<<"真值2："<<distance_[2]<< "距离："<<filteredDistance2<<std::endl;
        std::cout<<"真值0："<<distance_[0]<< "距离："<<filteredDistance0<<std::endl;

        // 判断距离是否偏离目标距离范围
        if (filteredDistance1 < targetDistance - distanceThreshold ) {
            // 距离过近，需要向左调整车辆行驶方向
            // 在这里添加调整车辆方向的代码
            std::cout<< "距离过近，left:"<<filteredDistance2 - targetDistance<<std::endl;
        } else if (filteredDistance2 > targetDistance + distanceThreshold) {
            // 距离过远，需要向左调整车辆行驶方向
            // 在这里添加调整车辆方向的代码
            std::cout<< "distance too far， right please!!"<<filteredDistance2 - targetDistance<<std::endl;
        } else {
            // 距离在目标范围内，维持当前行驶方向
            // 在这里添加维持当前行驶方向的代码
            std::cout<< "OK! go "<<std::endl;
        }
    }

    // int getDistance() {
    //     return distance_;
    // }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_monitor");
    ros::NodeHandle nh;

    DistanceSensor sensor;
    ros::spin();

    return 0;
}
