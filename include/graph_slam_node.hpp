#ifndef GRAPH_SLAM_HANDLER_H
#define GRAPH_SLAM_HANDLER_H

#include "ros/ros.h"
#include "ros/package.h"

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
#include "turtlebot_graph_slam/keyframe.h"

#include <vector>
#include <iostream>
#include <string>
#include <memory>
#include <exception>
#include <stdexcept>

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

    Pose2 current_pose_;

protected:
    ros::NodeHandle nh_;
    ros::Subscriber scan_match_sub_;

    ros::Publisher optimized_pose_pub_ = this->nh_.advertise<turtlebot_graph_slam::keyframe>("/graphslam/optimizedposes", 10);

public:
    int index_ = 0;

    bool initialValues_added_ = false;
    bool optimizer_initialised_ = false;
    bool graph_initialised_ = false;
    bool isam2InUse_;
    int current_scan_index_ = -1;

    bool saveGraph_ = true;

    graph_slam_handler(ros::NodeHandle &nh);
    ~graph_slam_handler();

    void initGraph();
    void initOptimizer();
    void addInitialValuestoGraph();
    void scanCB(const turtlebot_graph_slam::tfArrayConstPtr &scan_msg);
    void solveAndResetGraph();
    void publishResults();
};

graph_slam_handler::graph_slam_handler(ros::NodeHandle &nh) : nh_(nh)
{
    scan_match_sub_ = nh.subscribe("/scanmatches", 10, &graph_slam_handler::scanCB, this);

    initGraph();
    initOptimizer();

    nh_.getParam("/graphSLAM/saveGraph", saveGraph_);
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

void graph_slam_handler::addInitialValuestoGraph()
{
    double x = initial_Odom_.pose.pose.position.x;
    double y = initial_Odom_.pose.pose.position.y;

    tf::Quaternion q(initial_Odom_.pose.pose.orientation.x,
                     initial_Odom_.pose.pose.orientation.y,
                     initial_Odom_.pose.pose.orientation.z,
                     initial_Odom_.pose.pose.orientation.w);

    current_pose_ = Pose2(x, y, tf::getYaw(q));

    std::cout << " Prior pose  " << current_pose_ << std::endl;

    gtsam::Matrix33 prior_noise_model;
    prior_noise_model << initial_Odom_.pose.covariance[0], initial_Odom_.pose.covariance[1], initial_Odom_.pose.covariance[5],
        initial_Odom_.pose.covariance[6], initial_Odom_.pose.covariance[7], initial_Odom_.pose.covariance[11],
        initial_Odom_.pose.covariance[30], initial_Odom_.pose.covariance[31], initial_Odom_.pose.covariance[35];

    initial_estimates_.insert(X(index_), current_pose_);

    graph_->addPrior(X(index_), current_pose_, prior_noise_model);

    std::cout << " Prior noise model " << prior_noise_model << std::endl;

    initialValues_added_ = true;
}

void graph_slam_handler::scanCB(const turtlebot_graph_slam::tfArrayConstPtr &scan_msg)
{
    try
    {
        index_ = std::stoi(scan_msg->keyframe.child_frame_id);
    }
    catch (std::invalid_argument const &e)
    {
        ROS_ERROR("Error converting child_frame_id to integer: %s", e.what());
    }
    catch (std::out_of_range const &e)
    {
        ROS_ERROR("Error converting child_frame_id to integer: %s", e.what());
    }

    if (!initialValues_added_)
    {
        initial_Odom_ = scan_msg->keyframe;
        addInitialValuestoGraph();
        ROS_INFO_STREAM("Initial Values added to the graph.");
    }
    else
    {

        tf::Quaternion qu(scan_msg->keyframe.pose.pose.orientation.x,
                          scan_msg->keyframe.pose.pose.orientation.y,
                          scan_msg->keyframe.pose.pose.orientation.z,
                          scan_msg->keyframe.pose.pose.orientation.w);

        Pose2 odometry(scan_msg->keyframe.pose.pose.position.x, scan_msg->keyframe.pose.pose.position.y, tf::getYaw(qu));

        gtsam::Matrix33 odom_cov;
        odom_cov << scan_msg->keyframe.pose.covariance[0], scan_msg->keyframe.pose.covariance[1], scan_msg->keyframe.pose.covariance[5],
            scan_msg->keyframe.pose.covariance[6], scan_msg->keyframe.pose.covariance[7], scan_msg->keyframe.pose.covariance[11],
            scan_msg->keyframe.pose.covariance[30], scan_msg->keyframe.pose.covariance[31], scan_msg->keyframe.pose.covariance[35];

        if (graph_.get() != nullptr && optimizer_initialised_ && initialValues_added_ && graph_initialised_)
        {
            if (!scan_msg->transforms.empty())
            {
                for (int i = 0; i < scan_msg->transforms.size(); i++)
                {
                    // extract pose2 from the geometry_msgs/transformedStamped
                    tf::Quaternion qt(scan_msg->transforms[i].transform.rotation.x, scan_msg->transforms[i].transform.rotation.y, scan_msg->transforms[i].transform.rotation.z, scan_msg->transforms[i].transform.rotation.w);
                    Pose2 scan_pose(scan_msg->transforms[i].transform.translation.x, scan_msg->transforms[i].transform.translation.y, tf::getYaw(qt));

                    if (i == 0)
                    {
                        std::cout << "odometry pose - " << odometry << "  scan pose  " << scan_pose << std::endl;
                    }
                    int scan_match_with_frame = std::stoi(scan_msg->transforms[i].header.frame_id);

                    // add initial estimates composing logic
                    if (i == 0)
                    {
                        current_pose_ = current_pose_.compose(scan_pose);
                        initial_estimates_.insert(X(index_), current_pose_);

                        // add odom factor to the graph
                        graph_->emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(X(index_ - 1), X(index_), odometry, noiseModel::Diagonal::Covariance(odom_cov));
                    }

                    // if (scan_msg->covariances[i].data[0] > 1.0)
                    // {
                    //     ROS_ERROR_STREAM("Scan skipped in between ID " << index_ << "and " << scan_match_with_frame);
                    //     continue;
                    // }

                    // Add scan matching factor
                    if (scan_msg->covariances.empty() || scan_msg->covariances[i].data.size() != 9 || std::isnan(scan_msg->covariances[i].data[0]))
                    {
                        auto icp_noise_model = noiseModel::Diagonal::Sigmas(Vector3(0.09, 0.09, 0.01));
                        graph_->emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(X(scan_match_with_frame), X(index_), scan_pose, icp_noise_model);
                    }
                    else
                    {
                        std_msgs::Float64MultiArray covarianceArray = scan_msg->covariances[i];

                        // Convert the data to a std::vector<double>
                        std::vector<double> covarianceData;
                        covarianceData.assign(covarianceArray.data.begin(), covarianceArray.data.end());

                        // Initialize Matrix33 with the data
                        Matrix33 scan_cov;
                        scan_cov << covarianceData[0], covarianceData[1], covarianceData[2],
                            covarianceData[3], covarianceData[4], covarianceData[5],
                            covarianceData[6], covarianceData[7], covarianceData[8];

                        graph_->emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(X(scan_match_with_frame), X(index_), scan_pose, noiseModel::Diagonal::Covariance(scan_cov));
                    }
                }
            }
            else // there are no scan matches
            {
                current_pose_ = current_pose_.compose(odometry);

                // add initial estimates for X(1) from odometry
                initial_estimates_.insert(X(index_), current_pose_);

                // add odometry Factor between X(index_-1) to X(index_) to graph
                graph_->emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(X(index_ - 1), X(index_), odometry, noiseModel::Diagonal::Covariance(odom_cov));
            }

            if (index_ > 1)
            {
                solveAndResetGraph();
            }

            if (!results_.empty())
            {
                // update current pose
                current_pose_ = results_.at<Pose2>(X(index_));

                // Printing the results on console
                // Marginals marg(*graph_, results_);
                // for (int i = 0; i < results_.size(); i++)
                // {
                //     // ROS_INFO_STREAM("Keyframe number " << i << " Pose: - " << results_.at<Pose2>(X(i)));
                //     // ROS_INFO_STREAM("Keyframe number " << i << " Covariance: - \n" << marg.marginalCovariance(X(i)));
                // }
                // publish graph results
                publishResults();
            }
        }
    }
}

void graph_slam_handler::solveAndResetGraph()
{
    try
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
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Error occurred while optimizing the graph: " << e.what());
    }
    catch (...)
    {
        ROS_ERROR_STREAM("Unknown error occurred while optimizing the graph.");
    }

    if (saveGraph_)
    {
        std::string file_name = ros::package::getPath("turtlebot_graph_slam") + "/graph_viz/factorGraphViz.dot";
        ROS_INFO_STREAM("Graph viz file" << file_name);
        graph_->saveGraph(file_name, results_);
    }
}

void graph_slam_handler::publishResults()
{
    turtlebot_graph_slam::keyframe keyframe_msg;

    // Extract and publish optimized poses
    for (int i = 0; i < results_.size(); i++)
    {
        Pose2 pose = results_.at<Pose2>(X(i));
        geometry_msgs::Pose pose_stamped;
        pose_stamped.position.x = pose.x();
        pose_stamped.position.y = pose.y();
        pose_stamped.orientation = tf::createQuaternionMsgFromYaw(pose.theta());
        keyframe_msg.keyframePoses.poses.push_back(pose_stamped);
    }
    // If ISAM2 is not in use, add covariances
    if (!isam2InUse_)
    {
        std::vector<std_msgs::Float64MultiArray> covarianceMessages;
        Marginals marg(*graph_, results_);

        for (int i = 0; i < results_.size(); i++)
        {
            vector<double> covariance_vec(9);
            std_msgs::Float64MultiArray covarianceMsg;

            Matrix cov = marg.marginalCovariance(X(i));
            for (int j = 0; j < 3; j++)
            {
                for (int k = 0; k < 3; k++)
                {
                    covariance_vec[j * 3 + k] = cov(j, k);
                }
            }
            covarianceMsg.data.assign(covariance_vec.begin(), covariance_vec.end());
            covarianceMessages.push_back(covarianceMsg);
        }
        keyframe_msg.covariances.assign(covarianceMessages.begin(), covarianceMessages.end());
    }
    optimized_pose_pub_.publish(keyframe_msg);
}

#endif // GRAPH_SLAM_HANDLER_H
