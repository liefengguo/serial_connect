#include "../../include/radar_controller/RadarController.h"

RadarController::RadarController() {
    radar_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
    nh.param<int>("RadarController/targetDistance", targetDistance, 230);
    nh.param<int>("RadarController/distanceThreshold", distanceThreshold, 80);
    nh.param<int>("RadarController/angular_z", angular_z, 0.08);
    nh.param<int>("RadarController/threshold", linear_x, 0.3);
    nh.param<int>("RadarController/flag", flag, 1);
}

void RadarController::setGNSSStatus(int status) {
    gnss_status = status;
}

void RadarController::setRadar1(int value) {
    radar1 = value;
}

void RadarController::setRadar2(int value) {
    radar2 = value;
}

void RadarController::setRadar3(int value) {
    radar3 = value;
}

void RadarController::setRadar4(int value) {
    radar4 = value;
}

void RadarController::setRadar5(int value) {
    radar5 = value;
}

void RadarController::setRadar6(int value) {
    radar6 = value;
}

void RadarController::turnLeft() {
    vel_msg.angular.z = -angular_z;  // 设置负角速度以左转
    vel_msg.linear.x = linear_x;
    radar_cmd_vel.publish(vel_msg);
}
void RadarController::turnRight() {
    vel_msg.angular.z = angular_z;  // 设置正角速度以右转
    vel_msg.linear.x = linear_x;
    radar_cmd_vel.publish(vel_msg);
}
void RadarController::controlByRadar() {
    if (gnss_status < 7 || flag) {
        // RTK is good, use ultrasonic sensor for lateral control
        // Add your code here for controlling the vehicle using the filtered distance
        // 判断距离是否偏离目标距离范围
        if (radar2 < targetDistance - distanceThreshold ) {
            // std::cout<< "距离过近，left:"<<filteredDistance2 - targetDistance<<std::endl;
            turnLeft();
        } else if (radar2 > targetDistance + distanceThreshold) {
            turnRight();
            // std::cout<< "distance too far， right please!!"<<filteredDistance2 - targetDistance<<std::endl;
        } else {
            vel_msg.angular.z = 0;
            vel_msg.linear.x = linear_x;
            radar_cmd_vel.publish(vel_msg);
            // std::cout<< "OK! go "<<std::endl;
        }
    } else {
        // RTK is not good, use alternative control method
        // Add your code here for alternative control method
    }
}

