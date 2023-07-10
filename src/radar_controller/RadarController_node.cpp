#include "../../include/radar_controller/RadarController.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "radarController");
    VRFKReader vrtkReader;
    DistanceSensor sensor;
    RadarController radar_controller;
    while (ros::ok)
    {
        radar_controller.setRadar1(sensor.getFilteredDistance0());
        radar_controller.setRadar2(sensor.getFilteredDistance1());
        radar_controller.setRadar3(sensor.getFilteredDistance2());
        radar_controller.setRadar4(sensor.getFilteredDistance3());
        radar_controller.setRadar5(sensor.getFilteredDistance4());
        radar_controller.setRadar6(sensor.getFilteredDistance5());
        radar_controller.setGNSSStatus(vrtkReader.getGNSSStatus());
        radar_controller.controlByRadar();
        ros::spinOnce();
    }
    // ros::spin();
    return 0;
}
