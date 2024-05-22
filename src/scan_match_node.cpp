#include "ros/ros.h"
#include "scan_match_node.hpp"
#include<iostream>


int main(int argc, char **argv) {
    ros::init(argc, argv, "scan_match_node");
    ros::NodeHandle nh;

    ScanHandler handler(nh, 3.0, 0.7);
    
    ros::spin();
    return 0;
}