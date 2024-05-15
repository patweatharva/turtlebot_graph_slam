#ifndef SCAN_HANDLER_H
#define SCAN_HANDLER_H

#include <cmath>

// for ICP Covariance, contains Eigen Includes and PCL includes
#include "icp_cov_estimation.h"

// ROS includes
#include "ros/ros.h"
#include <ros/package.h>

// Message filter include
#include <message_filters/subscriber.h>

// tf includes
#include <tf/transform_listener.h> // Include tf header
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <laser_geometry/laser_geometry.h>

// Message types includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

// Services Includes
#include "turtlebot_graph_slam/ResetFilter.h"
#include "turtlebot_graph_slam/tfArray.h"
#include "turtlebot_graph_slam/keyframe.h"
#include <std_srvs/Empty.h>

// C++ includes
#include <sstream>
#include <iostream>
#include <vector>

// using namespace std;

template <typename T>
Eigen::Matrix4f TFtoSE3(const T &transform);

// Specialization for geometry_msgs::TransformStamped
template <>
Eigen::Matrix4f TFtoSE3<geometry_msgs::TransformStamped>(const geometry_msgs::TransformStamped &transformStamped)
{
    const geometry_msgs::Quaternion &q = transformStamped.transform.rotation;
    const geometry_msgs::Vector3 &t = transformStamped.transform.translation;

    Eigen::Quaternionf quaternion(q.w, q.x, q.y, q.z);
    Eigen::Vector3f translation(t.x, t.y, t.z);

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    transformation.block<3, 1>(0, 3) = translation;

    Eigen::Matrix4f transformation_inverse = transformation;

    return (transformation_inverse);
}

// Specialization for tf::StampedTransform
template <>
Eigen::Matrix4f TFtoSE3<tf::StampedTransform>(const tf::StampedTransform &stampedTransform)
{
    const tf::Quaternion &q = stampedTransform.getRotation();
    const tf::Vector3 &t = stampedTransform.getOrigin();

    Eigen::Quaternionf quaternion(q.w(), q.x(), q.y(), q.z());
    Eigen::Vector3f translation(t.x(), t.y(), t.z());

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    transformation.block<3, 1>(0, 3) = translation;

    Eigen::Matrix4f transformation_inverse = transformation.inverse();

    return (transformation_inverse);
}

Eigen::Matrix4f posetoSE3(const geometry_msgs::Pose &pose)
{
    Eigen::Matrix4f se3 = Eigen::Matrix4f::Identity();

    // Translation
    se3(0, 3) = pose.position.x;
    se3(1, 3) = pose.position.y;
    se3(2, 3) = pose.position.z;

    // Quaternion to rotation matrix
    Eigen::Quaternionf quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Matrix3f rotation = quat.toRotationMatrix();

    // Assign rotation matrix
    se3.block<3, 3>(0, 0) = rotation;

    return se3;
}

class ScanHandler
{
public:
    ros::NodeHandle &nh_;
    ros::Publisher pointcloud_pub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("/pc", 10);
    ros::Publisher keyframe_pub_ = this->nh_.advertise<geometry_msgs::PoseArray>("/keyframes", 10);
    ros::Publisher scan_match_pub_ = this->nh_.advertise<turtlebot_graph_slam::tfArray>("/scanmatches", 10);
    ros::Publisher ellipse_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("ellipse_markers", 1000);

    // Laser Geometry Projection for converting scan to PointCloud
    laser_geometry::LaserProjection projector_;

    // Initialising listener transform
    tf::TransformListener listener_;

    // message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    // tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;

    // Subsciber for optimized poses from graph SLAM
    ros::Subscriber op_keyframe_sub_;

    ros::Subscriber laser_sub_;

    // tf::TransformBroadcaster keyframe_br_;

    double thresholdTime_;
    double thresholdOdometry_;
    ros::Subscriber odom_sub_;
    ros::Timer timer_;

    bool odom_trigger_ = false;
    bool time_trigger_ = false;

    bool saveWorldMap_ = false;

    ros::Time lastScanTime_;
    nav_msgs::Odometry last_scan_odom_;
    nav_msgs::Odometry current_scan_odom_;
    nav_msgs::Odometry current_odom_;
    int current_scan_index = -1; // Frame -1 means map frame
    geometry_msgs::TransformStamped current_key_frame;
    ros::ServiceClient client_;
    ros::ServiceClient client_octomap_;

    Eigen::Matrix4f Twtok = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Tktokplus1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Twtokplus1 = Eigen::Matrix4f::Identity();

    ScanHandler(ros::NodeHandle &nh, double thresholdTime, double thresholdOdometry) : nh_(nh), thresholdTime_(thresholdTime), thresholdOdometry_(thresholdOdometry), client_(nh.serviceClient<turtlebot_graph_slam::ResetFilter>("ResetFilter")), client_octomap_(nh.serviceClient<std_srvs::Empty>("/octomap_server/reset"))
    {
        laser_sub_ = nh.subscribe("/turtlebot/kobuki/sensors/rplidar",10, &ScanHandler::scanCallback, this);

        // Initialize subscribers
        odom_sub_ = nh.subscribe("/odom", 10, &ScanHandler::odometryCallback, this);

        // Timer for getting a new scan
        timer_ = nh.createTimer(ros::Duration(thresholdTime_), &ScanHandler::timerCallback, this);

        op_keyframe_sub_ = nh.subscribe("/graphslam/optimizedposes", 10, &ScanHandler::keyframeCallback, this);

        nh_.getParam("/graphSLAM/saveWorldMap", saveWorldMap_);

        // Laser notifiers for the scans
        // laser_notifier_.registerCallback(boost::bind(&ScanHandler::scanCallback, this, _1));
        // laser_notifier_.setTolerance(ros::Duration(0.01));
        // lastScanTime_ = ros::Time::now();


    };

    void storePointCloudandKeyframe(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const geometry_msgs::TransformStamped &keyframe)
    {
        storedPointClouds_.push_back(cloud);
        storedKeyframes_.push_back(keyframe);
    };

    void storeWorldPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        map_storedPointClouds_.push_back(cloud);
    };

private:
    // A vector of Pointers to the PointClouds for storing Incoming scans
    // std::vector<pcl::PointCloud<pcl::PointXYZ>> storedPointClouds_;   // For directly storing pointclouds
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> storedPointClouds_;
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> map_storedPointClouds_;
    std::vector<geometry_msgs::TransformStamped> storedKeyframes_;
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> hypothesis_;
    std::vector<int> hypothesisIDs_;
    pcl::PointCloud<pcl::PointXYZ> world_map_;
    std::vector<Eigen::Matrix4f> keyframes_;
    std::vector<geometry_msgs::TransformStamped> Transformations_vector_;
    std::vector<std::vector<double>> covariances_;

    // Scan Callback method
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        if (time_trigger_ && odom_trigger_)
        {
            sensor_msgs::PointCloud2 cloud_in;
            try
            {
                ros::Time start_time_ = ros::Time::now();
                projector_.transformLaserScanToPointCloud("turtlebot/kobuki/base_footprint", *scan, cloud_in, listener_);
                // projector_.projectLaser(*scan,cloud_in);

                current_scan_odom_ = current_odom_;
                current_scan_odom_.header.frame_id = std::to_string(current_scan_index);
                current_scan_odom_.child_frame_id = std::to_string(current_scan_index + 1);

                ROS_ERROR_STREAM("SCAN TIME DIFFERENCE"<<  (current_scan_odom_.header.stamp - scan->header.stamp).toSec());
                // ROS_ERROR_STREAM("SCAN Heading DIFFERENCE"<<  (current_scan_odom_.pose.pose.orientation. - scan->header.stamp).toSec());

                odomtoTF(current_scan_odom_, current_key_frame);

                turtlebot_graph_slam::ResetFilter srv;
                srv.request.reset_filter_requested = true;

                if (client_.call(srv))
                {
                    ROS_INFO("Odometry Filter (EKF) is succesfully reset.... : %d", (bool)srv.response.reset_filter_response);
                }
                else
                {
                    ROS_ERROR("Failed to call reset Odometry Filter!!!!!!!");
                };

                Eigen::Matrix4f current_tf = TFtoSE3(current_key_frame);

                std::cout << "Current Key Frame:\n"
                          << current_tf << std::endl;

                // Convert to PCL format for processing
                pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(cloud_in, *pclCloud);

                // Only taking the planar PointCloud model
                pcl::PointCloud<pcl::PointXYZ>::Ptr planarPointcloud = fitPlanarModel(pclCloud);

                // Storing the Pointcloud and keyframe
                storePointCloudandKeyframe(planarPointcloud, current_key_frame); // Pass the PCL point cloud pointer
                current_scan_index++;
                ROS_INFO("---Pointcloud (ID - %d) Received and Stored---", current_scan_index);

                // Publishing Obtained Pointcloud
                // pointcloud_pub_.publish(cloud_in);

                if (current_scan_index > 0)
                {
                    Eigen::Matrix4f TMaptoCurrent = keyframes_.back() * current_tf;
                    // Hypothesis generation and matching
                    pclMatchHypothesis(TMaptoCurrent);
                    Transformations_vector_ = pointCloudMatching(planarPointcloud);
                    ROS_INFO("---Factors for Pointcloud (ID - %d) Calculated---", current_scan_index);
                    // ROS_INFO("SIZE of the transformations_vector -- %zu ", Transformations_vector.size());
                    for (const auto &transform : Transformations_vector_)
                    {
                        double theta = tf::getYaw(transform.transform.rotation);
                        ROS_INFO("Transformation : (%f,%f,%f)",
                                 transform.transform.translation.x,
                                 transform.transform.translation.y,
                                 theta);
                    };
                }
                else
                {
                    Twtok = TFtoSE3(current_key_frame);
                }
                ros::Time end_time_ = ros::Time::now();
                ros::Duration scan_processing_time = end_time_ - start_time_;

                ROS_INFO_STREAM(" Scan Processing Runtime: " << scan_processing_time.toSec());

                publishMatching(Transformations_vector_, covariances_);

                // TODO: Registrations conditions
                if (true || current_scan_index == 0 || current_scan_index == 1 || current_scan_index == 2 || current_scan_index == 3)
                {
                    registerPointcloudinWorld(planarPointcloud);
                }

                time_trigger_ = false;
                odom_trigger_ = false;
                last_scan_odom_ = current_scan_odom_;
            }
            catch (tf::TransformException &e)
            {
                ROS_ERROR("Transform error: %s", e.what());
                return;
            };
        }
        else
        {
            return;
        };
    };

    // Odometry Callback
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &odom)
    {
        // Saving current odometry
        current_odom_ = *odom;
        tf::Quaternion q(current_odom_.pose.pose.orientation.x,
                         current_odom_.pose.pose.orientation.y,
                         current_odom_.pose.pose.orientation.z,
                         current_odom_.pose.pose.orientation.w);

        double yaw = tf::getYaw(q);

        if (fabs(yaw) > M_PI / 20 || fabs(current_odom_.pose.pose.position.x) >= 0.1 || fabs(current_odom_.pose.pose.position.y) >= 0.1)
        {
            odom_trigger_ = true;
        }
        else
        {
            odom_trigger_ = false;
        }
    };

    // Timer Callback
    void timerCallback(const ros::TimerEvent &)
    {
        time_trigger_ = true;
    };

    pcl::PointCloud<pcl::PointXYZ>::Ptr fitPlanarModel(pcl::PointCloud<pcl::PointXYZ>::Ptr &currentScan)
    {

        // Check if the points are planar
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(currentScan);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            // No planar model found
            ROS_INFO("NO Planar model found for the current scan !!");
        }

        // Extract only the inliers from the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr inlierCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(currentScan);
        extract.setIndices(inliers);
        extract.setNegative(false); // Set to true to get outliers
        extract.filter(*inlierCloud);

        return inlierCloud;
    };

    void pclMatchHypothesis(Eigen::Matrix4f &TMaptoCurrent)
    {
        // Clear Previous Hypothesis
        hypothesis_.clear();
        hypothesisIDs_.clear();

        hypothesis_.push_back(storedPointClouds_[storedPointClouds_.size() - 2]);
        hypothesisIDs_.push_back(storedPointClouds_.size() - 2);

        for (int i = 0; i < keyframes_.size() - 1; i++)
        {
            // Compute the Euclidean distance between translations
            float distance = std::sqrt(
                std::pow(TMaptoCurrent(0, 3) - keyframes_[i](0, 3), 2) +
                std::pow(TMaptoCurrent(1, 3) - keyframes_[i](1, 3), 2) +
                std::pow(TMaptoCurrent(2, 3) - keyframes_[i](2, 3), 2));

            // Compare distance with thresholdOdometry_
            if (distance < thresholdOdometry_ && hypothesisIDs_.size() <= 6)
            {
                hypothesis_.push_back(storedPointClouds_[i]);
                hypothesisIDs_.push_back(i);
            }
        }
    };

    Eigen::Matrix4f getInitialGuses(Eigen::Matrix4f &TMaptoCurrent, Eigen::Matrix4f &hypothesis)
    {
        return (hypothesis.inverse()) * TMaptoCurrent;
    }

    geometry_msgs::TransformStamped SE3toTF(const Eigen::Matrix4f &transformation, const std::string &frame_id, const std::string &child_frame_id)
    {
        // Extract the rotation matrix and translation vector from the transformation matrix
        Eigen::Matrix3f rotationMatrix = transformation.block<3, 3>(0, 0);
        Eigen::Vector3f translationVector = transformation.block<3, 1>(0, 3);

        // Convert the rotation matrix to a quaternion
        Eigen::Quaternionf quaternion(rotationMatrix);

        // Construct the Transform message
        geometry_msgs::Transform transform;
        transform.translation.x = translationVector.x();
        transform.translation.y = translationVector.y();
        transform.translation.z = translationVector.z();
        transform.rotation.x = quaternion.x();
        transform.rotation.y = quaternion.y();
        transform.rotation.z = quaternion.z();
        transform.rotation.w = quaternion.w();

        // Create a TransformStamped message
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.transform = transform;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame_id;
        transformStamped.header.stamp = ros::Time::now();

        return transformStamped;
    };

    void odomtoTF(const nav_msgs::Odometry &odom_in, geometry_msgs::TransformStamped &tf_out)
    {
        // Extract position and orientation from the Odometry message
        tf_out.header.stamp = odom_in.header.stamp;
        tf_out.header.frame_id = std::to_string(current_scan_index);    // Parent frame
        tf_out.child_frame_id = std::to_string(current_scan_index + 1); // Child frame

        // Position (translation)
        tf_out.transform.translation.x = odom_in.pose.pose.position.x;
        tf_out.transform.translation.y = odom_in.pose.pose.position.y;
        tf_out.transform.translation.z = odom_in.pose.pose.position.z;

        // Orientation (rotation)
        tf_out.transform.rotation.x = odom_in.pose.pose.orientation.x;
        tf_out.transform.rotation.y = odom_in.pose.pose.orientation.y;
        tf_out.transform.rotation.z = odom_in.pose.pose.orientation.z;
        tf_out.transform.rotation.w = odom_in.pose.pose.orientation.w;
    }

    std::vector<geometry_msgs::TransformStamped> pointCloudMatching(const pcl::PointCloud<pcl::PointXYZ>::Ptr &currentScan)
    {
        std::vector<geometry_msgs::TransformStamped> transformations;
        Eigen::Matrix4f finalTF = Eigen::Matrix4f::Identity();
        geometry_msgs::TransformStamped tfs;
        covariances_.clear();

        ROS_INFO_STREAM("Size of the hypothesis " << hypothesisIDs_.size());

        for (size_t i = 0; i < hypothesis_.size(); ++i)
        {
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(currentScan);
            icp.setInputTarget(hypothesis_[i]);

            // Setup for the Correspondence Estimation
            pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>::Ptr corr_est(new pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>);
            corr_est->setInputSource(currentScan);
            corr_est->setInputTarget(hypothesis_[i]);
            pcl::Correspondences corr_map;
            corr_est->determineReciprocalCorrespondences(corr_map, 0.05);

            icp.setCorrespondenceEstimation(corr_est);
            icp.setUseReciprocalCorrespondences(true);

            // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
            icp.setMaxCorrespondenceDistance(0.05);

            // Set the maximum number of iterations
            icp.setMaximumIterations(5000);

            // Set the transformation epsilon
            icp.setTransformationEpsilon(1e-6);

            // Set the euclidean distance difference epsilon
            icp.setEuclideanFitnessEpsilon(0.05);

            // Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f initial_guess = TFtoSE3(current_key_frame);
            Eigen::Matrix4f TMaptoCurrent = keyframes_.back() * initial_guess;
            initial_guess = getInitialGuses(TMaptoCurrent, keyframes_[hypothesisIDs_[i]]);

            std::cout << " Initial guess matrix current " << current_scan_index << "and " << hypothesisIDs_[i] << ":\n"
                      << initial_guess << std::endl;

            // Create an output point cloud for the aligned source
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_aligned(new pcl::PointCloud<pcl::PointXYZ>);

            // Compute the transformation
            icp.align(*cloud_source_aligned, initial_guess);

            // Obtain the final transformation matrix
            if (hypothesisIDs_[i] == (current_scan_index - 1))
            {
                Tktokplus1 = icp.getFinalTransformation();
                getICPCov(cloud_source_aligned, hypothesis_[i], Tktokplus1, corr_map);
            }
            else
            {
                finalTF = icp.getFinalTransformation();
                getICPCov(cloud_source_aligned, hypothesis_[i], finalTF, corr_map);
            }

            if (icp.hasConverged())
            {
                double meanSquaredDistance = (double)icp.getFitnessScore();
                ROS_INFO("ICP Fitness score --- %f", meanSquaredDistance);

                // For checking scan Matching
                // if (current_scan_index < 10)
                // {
                //     savePointcloud(currentScan, hypothesis_[i], cloud_source_aligned , std::to_string(hypothesisIDs_[i]), std::to_string(current_scan_index));
                // };

                if (meanSquaredDistance <= 1.5)
                {
                    // Create a TransformStamped message
                    std::string frameID = std::to_string(hypothesisIDs_[i]);
                    std::string childID = std::to_string(current_scan_index);
                    if (hypothesisIDs_[i] == (current_scan_index - 1))
                    {
                        tfs = SE3toTF(Tktokplus1, frameID, childID);
                    }
                    else
                    {
                        tfs = SE3toTF(finalTF, frameID, childID);
                    }
                    // Add the transformation to the vector
                    transformations.push_back(tfs);
                };
            };
        };
        return transformations;
    };

    void getICPCov(const pcl::PointCloud<pcl::PointXYZ>::Ptr &aligned, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, Eigen::Matrix4f &finalTF, pcl::Correspondences &corr_map)
    {

        std::vector<int> data_idx;
        std::vector<int> model_idx;

        pcl::Correspondence temp1;
        for (int i = 0; i < corr_map.size(); i++)
        {
            temp1 = corr_map[i];
            data_idx.push_back(temp1.index_query);
            model_idx.push_back(temp1.index_match);
        }

        Eigen::MatrixXd ICP_COV(6, 6);
        ICP_COV = Eigen::MatrixXd::Zero(6, 6);

        pcl::PointCloud<pcl::PointXYZ> data_pi;
        pcl::PointCloud<pcl::PointXYZ> model_qi;

        pcl::copyPointCloud(*aligned, data_idx, data_pi);
        pcl::copyPointCloud(*target, model_idx, model_qi);

        calculate_ICP_COV(data_pi, model_qi, finalTF, ICP_COV);

        ROS_INFO_STREAM("ICP Covariance: " << ICP_COV);

        // Add covariance in covariances_
        std::vector<double> cov_vec{ICP_COV(0, 0), ICP_COV(0, 1), ICP_COV(0, 3), ICP_COV(1, 0), ICP_COV(1, 1), ICP_COV(1, 3), ICP_COV(3, 0), ICP_COV(3, 1), ICP_COV(3, 3)};

        covariances_.push_back(cov_vec);
    }

    void savePointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, const pcl::PointCloud<pcl::PointXYZ>::Ptr &aligned, const std::string &target_id, const std::string &source_id)
    {
        // Get the path to the current ROS package
        std::string packagePath = ros::package::getPath("turtlebot_graph_slam");
        std::string directory = "/pcl_viz/";
        pcl::io::savePCDFileASCII(packagePath + directory + source_id + "-"+ target_id + "source.pcd", *source);
        pcl::io::savePCDFileASCII(packagePath + directory + source_id + "-"+ target_id + "target.pcd", *target);
        pcl::io::savePCDFileASCII(packagePath + directory + source_id + "-"+ target_id + "aligned.pcd", *aligned);
    };

    void registerPointcloudinWorld(const pcl::PointCloud<pcl::PointXYZ>::Ptr &currentScan)
    {

        // std::cout << "T w to k for scan ID " << current_scan_index << ":\n"
        //           << Twtok << std::endl;
        // std::cout << "T k to k+1 for scan ID " << current_scan_index << ":\n"
        //           << Tktokplus1 << std::endl;

        Twtokplus1 = Twtok * Tktokplus1;

        keyframes_.push_back(Twtokplus1);

        // std::cout << "T w to k+1 for scan ID " << current_scan_index << ":\n"
        //           << Twtokplus1 << std::endl;

        // publishKeyframeinMap();

        // Transform currentScan to map frame
        // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_world(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::transformPointCloud(*currentScan, *transformed_cloud_world, Twtokplus1);

        // // Store it in map_storedPointClouds_
        // storeWorldPointCloud(transformed_cloud_world);

        // // Combine the pointcloud
        // if (world_map_.empty())
        // {
        //     world_map_ = (*transformed_cloud_world);
        // }
        // else
        // {

        //     world_map_ += (*transformed_cloud_world);
        // };

        Twtok = Twtokplus1;

        // Publish the whole point cloud
        // publishWorldPointcloud();
    };

    void publishWorldPointcloud()
    {
        sensor_msgs::PointCloud2 world_cloud_out;

        pcl::toROSMsg(world_map_, world_cloud_out);

        world_cloud_out.header.frame_id = "map";
        world_cloud_out.header.stamp = ros::Time::now();

        // Publish the point cloud
        pointcloud_pub_.publish(world_cloud_out);
    };

    void publishKeyframeinMap()
    {
        geometry_msgs::PoseArray poseArray;
        poseArray.header.frame_id = "map";
        poseArray.header.stamp = ros::Time::now();

        // Iterate over all keyframes
        for (size_t i = 0; i < keyframes_.size(); ++i)
        {
            geometry_msgs::Pose pose;
            pose.position.x = keyframes_[i](0, 3);
            pose.position.y = keyframes_[i](1, 3);
            pose.position.z = keyframes_[i](2, 3);

            Eigen::Quaternionf quaternion(keyframes_[i].block<3, 3>(0, 0));
            pose.orientation.x = quaternion.x();
            pose.orientation.y = quaternion.y();
            pose.orientation.z = quaternion.z();
            pose.orientation.w = quaternion.w();

            // Add the pose to the PoseArray
            poseArray.poses.push_back(pose);
        }

        keyframe_pub_.publish(poseArray);
    }

    void publishMatching(const std::vector<geometry_msgs::TransformStamped> &transforms, const std::vector<std::vector<double>> &covariances)
    {
        turtlebot_graph_slam::tfArray tfArray_msg;
        tfArray_msg.transforms.assign(transforms.begin(), transforms.end());

        // // TODO: Adding ICP covariances into the msg
        // std::vector<std_msgs::Float64MultiArray> covarianceMessages;

        // for (const auto &cov : covariances)
        // {
        //     std_msgs::Float64MultiArray covarianceMsg;
        //     covarianceMsg.data.assign(cov.begin(), cov.end());
        //     covarianceMessages.push_back(covarianceMsg);
        // }
        // tfArray_msg.covariances.assign(covarianceMessages.begin(), covarianceMessages.end());

        tfArray_msg.keyframe = current_scan_odom_;

        scan_match_pub_.publish(tfArray_msg);
    }

    void keyframeCallback(const turtlebot_graph_slam::keyframe::ConstPtr &kfs)
    {

        // Clear previous map before building new map with optimized poses
        world_map_.clear();

        std_srvs::Empty empt_srv;

        if (client_octomap_.call(empt_srv))
        {
            ROS_INFO("Octomap reset!");
        }
        else
        {
            ROS_ERROR("Failed to call reset Octomap!!!!!!!");
        };

        // Extract and Update all keyframe poses
        for (int i = 0; i < (kfs->keyframePoses.poses.size()); i++)
        {
            Eigen::Matrix4f keyframeSE3 = posetoSE3(kfs->keyframePoses.poses[i]);
            keyframes_[i] = keyframeSE3;
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_world(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*storedPointClouds_[i], *transformed_cloud_world, keyframeSE3);
            storeWorldPointCloud(transformed_cloud_world);
            // Combine the pointcloud
            if (world_map_.empty())
            {
                world_map_ = (*transformed_cloud_world);
            }
            else
            {
                world_map_ += (*transformed_cloud_world);
            };
        }

        if (saveWorldMap_)
        {
            std::string packagePath = ros::package::getPath("turtlebot_graph_slam");
            std::string directory = "/pcl_viz/";
            pcl::io::savePCDFileASCII(packagePath + directory + "world_map.pcd", world_map_);
        }

        visualization_msgs::MarkerArray markerArray;

        // if kfs->covariances is not empty then extract covariances create ellipse markers and publish
        if (!kfs->covariances.empty())
        {
            // Loop through each covariance matrix
            for (int i = 0; i < kfs->covariances.size(); i++)
            {
                // Convert the covariance matrix from Float64MultiArray to Eigen Matrix
                Eigen::Matrix<float, 3, 3> covarianceMatrix;
                for (int j = 0; j < 3; ++j)
                {
                    for (int k = 0; k < 3; ++k)
                    {
                        covarianceMatrix(j, k) = kfs->covariances[i].data[j * 3 + k];
                    }
                }

                Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver(covarianceMatrix);
                Eigen::VectorXf eigenvalues = eigensolver.eigenvalues();
                Eigen::MatrixXf eigenvectors = eigensolver.eigenvectors();

                double lengthMajor = std::sqrt(eigenvalues(2));
                double lengthMinor = std::sqrt(eigenvalues(1));

                // ROS_INFO_STREAM("Covariance Matrix "<< covarianceMatrix);
                // ROS_INFO_STREAM("eigen vectors "<< eigenvectors);
                // ROS_INFO_STREAM("Eigen values "<< eigenvalues);

                tf::Quaternion q;
                q.setRPY(0, 0, atan2(eigenvectors(0, 0), eigenvectors(1, 0)));

                tf::Pose kfpose;
                tf::poseMsgToTF(kfs->keyframePoses.poses[i], kfpose);

                visualization_msgs::Marker marker;
                marker.id = i;
                marker.header.frame_id = "map";

                marker.type = visualization_msgs::Marker::CYLINDER;

                marker.pose.position.x = kfpose.getOrigin().x();
                marker.pose.position.y = kfpose.getOrigin().y();
                marker.pose.position.z = 0.0;

                marker.pose.orientation.w = q.w();
                marker.pose.orientation.x = q.x();
                marker.pose.orientation.y = q.y();
                marker.pose.orientation.z = q.z();

                marker.scale.x = lengthMajor;
                marker.scale.y = lengthMinor;
                marker.scale.z = 0.001;

                marker.color.a = 0.5; // Fully opaque
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;

                markerArray.markers.push_back(marker);
            }
        }

        publishKeyframeinMap();
        publishWorldPointcloud();
        publishCovEllipse(markerArray);
    }

    void publishCovEllipse(const visualization_msgs::MarkerArray &ellipse_arr)
    {
        if (!ellipse_arr.markers.empty())
        {
            ellipse_pub_.publish(ellipse_arr);
        }
    }
};

#endif // SCAN_HANDLER_H