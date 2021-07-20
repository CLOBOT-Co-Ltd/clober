#include <iostream>
#include <ros/ros.h>
#include <clober_serial/clober_serial.hpp>


int main(int argc, char** argv){

    ros::init(argc, argv, "clober_serial_node");

    CloberSerial driver;

    ros::spin();

    return 0;
}