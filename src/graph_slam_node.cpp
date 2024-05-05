#include "ros/ros.h"
#include "graph_slam_node.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "graph_slam_node");
    ros::NodeHandle nh;

    graph_slam_handler handler(nh);

    ros::spin();
}