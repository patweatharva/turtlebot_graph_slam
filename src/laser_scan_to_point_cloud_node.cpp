#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

class LaserScanToPointCloud
{
public:
    LaserScanToPointCloud()
        : tfListener(tfBuffer),
          scan_sub_(nh_, "/turtlebot/kobuki/sensors/rplidar", 1),
          tf_filter_(scan_sub_, tfBuffer, "turtlebot/kobuki/base_footprint", 10, nh_)
    {
        tf_filter_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sensors/pointcloud", 1);
    }

    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
    {
        sensor_msgs::PointCloud2 cloud;
        try
        {
            laser_geometry::LaserProjection projector_;
            projector_.transformLaserScanToPointCloud("map", *scan_msg, cloud, tfBuffer);
            point_cloud_pub_.publish(cloud);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Transform failure: %s", ex.what());
        }
    }

private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    tf2_ros::MessageFilter<sensor_msgs::LaserScan> tf_filter_;
    ros::Publisher point_cloud_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_scan_to_point_cloud");
    LaserScanToPointCloud lstopc;
    ros::spin();
    return 0;
}