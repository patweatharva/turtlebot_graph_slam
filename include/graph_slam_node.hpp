#ifndef GRAPH_SLAM_HANDLER_H
#define GRAPH_SLAM_HANDLER_H

#include "ros/ros.h"

#include <Eigen/Dense>

#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

// GTSAM related includes.
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/base/Value.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>

#include <gtsam/nonlinear/Marginals.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>

#include "turtlebot_graph_slam/tfArray.h"

#include <vector>
#include <iostream>
#include <memory>

using namespace std;
using namespace gtsam;

using symbol_shorthand::X; // Pose2 state (x,y,theta)

class graph_slam_handler
{
private:
    std::unique_ptr<NonlinearFactorGraph> graph_;
    std::unique_ptr<ISAM2> isam2_;

    ISAM2Params isam2params_;
    LevenbergMarquardtParams optimizerParams_;

    Values initial_estimates_;
    Vector3 initial_sigmas_;
    Values results_;

    nav_msgs::Odometry initial_Odom_;
    nav_msgs::Odometry current_odom_;

protected:
    ros::NodeHandle nh_;
    ros::Subscriber scan_match_sub_, odom_sub_;

public:
    int index_ = -1;

    bool initialValues_added_ = false;
    bool optimizer_initialised_ = false;
    bool graph_initialised_ = false;
    bool isam2InUse_;

    graph_slam_handler(ros::NodeHandle &nh);
    ~graph_slam_handler();

    void initGraph();
    void initOptimizer();
    void addInitialValuestoGraph();
    void odomCB(const nav_msgs::Odometry::ConstPtr &odom_msg);
    void scanCB(const turtlebot_graph_slam::tfArrayConstPtr &scan_msg);
    void solveAndResetGraph();
};

graph_slam_handler::graph_slam_handler(ros::NodeHandle &nh) : nh_(nh)
{
    // scan_match_sub_ = nh.subscribe("/scanmatches", 10, &graph_slam_handler::scanCB, this);
    odom_sub_ = nh.subscribe("/odom", 1, &graph_slam_handler::odomCB, this);

    initGraph();
    initOptimizer();
}

graph_slam_handler::~graph_slam_handler()
{
    return;
}

void graph_slam_handler::initGraph()
{
    if (graph_ == nullptr)
    {
        graph_ = std::make_unique<NonlinearFactorGraph>();
        ROS_INFO_STREAM("Nonlinear Factor Graph Initialized!!");
        graph_initialised_ = true; 
    }
}

void graph_slam_handler::initOptimizer()
{
    if (nh_.getParam("/graphSLAM/ISAM2/Use_ISAM2", isam2InUse_))
    {
        if (isam2InUse_)
        {
            // Fetch ISAM2 parameters
            double relinearizeThreshold;  // TODO: Tuning
            int relinearizeSkip;          // TODO: Tuning
            std::string factorizationStr; // TODO: Tuning

            if (!nh_.getParam("/graphSLAM/ISAM2/params/relinearizeThreshold", relinearizeThreshold))
            {
                ROS_ERROR("Failed to get ISAM2 relinearizeThreshold parameter");
                return;
            }
            if (!nh_.getParam("/graphSLAM/ISAM2/params/relinearizeSkip", relinearizeSkip))
            {
                ROS_ERROR("Failed to get ISAM2 relinearizeSkip parameter");
                return;
            }
            if (!nh_.getParam("/graphSLAM/ISAM2/params/factorization", factorizationStr))
            {
                ROS_ERROR("Failed to get ISAM2 factorization parameter");
                return;
            }

            // Convert string to gtsam::ISAM2Params::Factorization
            gtsam::ISAM2Params::Factorization factorization;
            if (factorizationStr == "QR")
            {
                factorization = gtsam::ISAM2Params::Factorization::QR;
            }
            else if (factorizationStr == "CHOLESKY")
            {
                factorization = gtsam::ISAM2Params::Factorization::CHOLESKY;
            }
            else
            {
                ROS_ERROR("Invalid factorization type: %s", factorizationStr.c_str());
                return;
            }

            // Initialize ISAM2 with fetched parameters
            isam2params_.factorization = factorization;
            isam2params_.relinearizeSkip = relinearizeSkip;
            isam2params_.setRelinearizeThreshold(relinearizeThreshold);

            // Create ISAM2 instance
            isam2_ = std::make_unique<ISAM2>(isam2params_);
            ROS_INFO_STREAM("ISAM2 Optimizer initialized with parameters!!!!");
        }
        else
        {
            ROS_INFO("ISAM2 Not Initialized!! Using LevenbergMarquardt Optimizer!!");

            // Read LevenbergMarquardt parameters
            double initialLambda, lambdaFactor, lambdaUpperBound, lambdaLowerBound, minModelFidelity, diagonalDamping;
            bool useDiagonalDamping, useLevenbergMarquardt;

            if (!nh_.getParam("/graphSLAM/LevenbergMarquardt/params/initialLambda", initialLambda))
            {
                ROS_ERROR("Failed to get LevenbergMarquardt initialLambda parameter");
                return;
            }
            if (!nh_.getParam("/graphSLAM/LevenbergMarquardt/params/lambdaFactor", lambdaFactor))
            {
                ROS_ERROR("Failed to get LevenbergMarquardt lambdaFactor parameter");
                return;
            }
            if (!nh_.getParam("/graphSLAM/LevenbergMarquardt/params/lambdaUpperBound", lambdaUpperBound))
            {
                ROS_ERROR("Failed to get LevenbergMarquardt lambdaUpperBound parameter");
                return;
            }
            if (!nh_.getParam("/graphSLAM/LevenbergMarquardt/params/lambdaLowerBound", lambdaLowerBound))
            {
                ROS_ERROR("Failed to get LevenbergMarquardt lambdaLowerBound parameter");
                return;
            }
            if (!nh_.getParam("/graphSLAM/LevenbergMarquardt/params/minModelFidelity", minModelFidelity))
            {
                ROS_ERROR("Failed to get LevenbergMarquardt minModelFidelity parameter");
                return;
            }
            if (!nh_.getParam("/graphSLAM/LevenbergMarquardt/params/useDiagonalDamping", useDiagonalDamping))
            {
                ROS_ERROR("Failed to get LevenbergMarquardt useDiagonalDamping parameter");
                return;
            }
            if (!nh_.getParam("/graphSLAM/LevenbergMarquardt/params/diagonalDamping", diagonalDamping))
            {
                ROS_ERROR("Failed to get LevenbergMarquardt diagonalDamping parameter");
                return;
            }
            if (!nh_.getParam("/graphSLAM/LevenbergMarquardt/params/useLevenbergMarquardt", useLevenbergMarquardt))
            {
                ROS_ERROR("Failed to get LevenbergMarquardt useLevenbergMarquardt parameter");
                return;
            }

            optimizerParams_.setlambdaInitial(initialLambda);        // TODO: Tuning
            optimizerParams_.setlambdaFactor(lambdaFactor);          // TODO: Tuning
            optimizerParams_.setlambdaUpperBound(lambdaUpperBound);  // TODO: Tuning
            optimizerParams_.setlambdaLowerBound(lambdaLowerBound);  // TODO: Tuning
            optimizerParams_.setDiagonalDamping(useDiagonalDamping); // TODO: Tuning

            ROS_INFO_STREAM("LevenbergMarquardt Optimizer initialized with parameters!!");
        }
        optimizer_initialised_ = true;
    }
    else
    {
        ROS_ERROR("Failed to get Use_ISAM2 parameter");
    }
}

void graph_slam_handler::odomCB(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    current_odom_ = *odom_msg;
    if (!initialValues_added_ && graph_initialised_)
    {
        initial_Odom_ = *odom_msg;
        addInitialValuestoGraph();
        initialValues_added_ = true;
        ROS_INFO_STREAM("Initial Values added to the graph.");
    }
}

void graph_slam_handler::addInitialValuestoGraph()
{
    double x = initial_Odom_.pose.pose.position.x;
    double y = initial_Odom_.pose.pose.position.y;

    tf::Quaternion q(initial_Odom_.pose.pose.orientation.x,
                     initial_Odom_.pose.pose.orientation.y,
                     initial_Odom_.pose.pose.orientation.z,
                     initial_Odom_.pose.pose.orientation.w);

    Pose2 prior_pose(x, y, tf::getYaw(q));
    std::cout << " Prior pose  " << prior_pose << std::endl;

    Eigen::Matrix3d prior_noise_model;
    prior_noise_model << initial_Odom_.pose.covariance[0], initial_Odom_.pose.covariance[1], initial_Odom_.pose.covariance[5],
        initial_Odom_.pose.covariance[6], initial_Odom_.pose.covariance[7], initial_Odom_.pose.covariance[11],
        initial_Odom_.pose.covariance[30], initial_Odom_.pose.covariance[31], initial_Odom_.pose.covariance[35];

    initial_estimates_.insert(X(index_), prior_pose);

    graph_->addPrior(X(index_), prior_pose, prior_noise_model);

    std::cout << " Prior noise model " << prior_noise_model << std::endl;
}

void graph_slam_handler::scanCB(const turtlebot_graph_slam::tfArrayConstPtr &scan_msg)
{
    if (graph_.get() != nullptr && optimizer_initialised_ && initialValues_added_ && graph_initialised_)
    {
        if (scan_msg->transforms.empty())
        {
            index_++;
            if ((index_) == 0) // Initial frame (0th frame)
            {
                
            }
            else // No good matching found
            {

            }
        }
        else
        {
        }
    }
}

void graph_slam_handler::solveAndResetGraph()
{
    if (isam2InUse_)
    {
        ROS_INFO_STREAM("Optimizing the graph using ISAM2!");

        isam2_->update(*graph_, initial_estimates_);
        results_ = isam2_->calculateEstimate();

        graph_->resize(0);
        initial_estimates_.clear();
    }
    else
    {
        ROS_INFO_STREAM("Optimizing the graph using Levenberg-Marquardt Optimizer!");

        // Create LevenbergMarquardtOptimizer instance
        LevenbergMarquardtOptimizer optimizer(*graph_, initial_estimates_, optimizerParams_);
        results_ = optimizer.optimize();
    }
}

#endif // GRAPH_SLAM_HANDLER_H
