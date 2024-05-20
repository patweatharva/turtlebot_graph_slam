# turtlebot_graph_slam

This ROS package implements Graph SLAM (Simultaneous Localization and Mapping) for the Turtlebot robot.

## Overview

Graph SLAM is a technique used in robotics to estimate the robot's trajectory and create a map of the environment simultaneously. This package utilizes the Turtlebot robot to perform Graph SLAM using sensors - 2D Lidar, Wheel Encoders, and Magnetometer Compass.

## Dependencies

This package has the following dependencies:

- ROS (Robot Operating System)
- Turtlebot packages
- GMapping package

## Installation

To install the turtlebot_graph_slam package, follow these steps:

1. Clone the repository into your ROS workspace:
    ```
    $ git clone https://github.com/your-username/turtlebot_graph_slam.git
    ```

2. Build the package:
    ```
    $ cd ~/catkin_ws
    $ catkin_make
    ```

## Usage

To use the turtlebot_graph_slam package, follow these steps:

1. Launch the Turtlebot robot:
    ```
    $ roslaunch turtlebot_bringup minimal.launch
    ```

2. Launch the GMapping package:
    ```
    $ roslaunch turtlebot_navigation gmapping.launch
    ```

3. Run the turtlebot_graph_slam node:
    ```
    $ rosrun turtlebot_graph_slam turtlebot_graph_slam_node
    ```

## Parameters

The turtlebot_graph_slam package provides the following parameters:

- `scan_topic` (string): The topic name for the laser scan data. Default: `/scan`
- `odom_topic` (string): The topic name for the odometry data. Default: `/odom`
- `map_frame` (string): The frame ID for the map. Default: `map`
- `base_frame` (string): The frame ID for the base link of the robot. Default: `base_link`

## Launch Files

The turtlebot_graph_slam package provides the following launch files:

- `turtlebot_graph_slam.launch`: Launches the turtlebot_graph_slam node with default parameters.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.