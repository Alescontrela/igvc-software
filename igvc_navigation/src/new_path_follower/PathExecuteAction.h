#ifndef PATHEXECUTEACTION_H
#define PATHEXECUTEACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <igvc_msgs/PathExecuteAction.h>
#include <igvc_msgs/velocity_pair.h>
#include <igvc_msgs/GeneratePath.h>

#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <memory>
#include <cstdio>
#include "SmoothControl.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

class PathExecuteAction
{
public:
    PathExecuteAction();
    PathExecuteAction(std::string name);
    ~PathExecuteAction(void);

    void map_callback(const igvc_msgs::mapConstPtr& msg);

    void goalCB();
    void preemptCB();
    void pathExecuteCB(const nav_msgs::OdometryConstPtr& msg);

    void load_new_path();
    void load_new_trajectory(const nav_msgs::OdometryConstPtr& msg);

protected:
    ros::NodeHandle nh_; // nodehandle

    // action server
    actionlib::SimpleActionServer<igvc_msgs::PathExecuteAction> as_;
    std::string action_name_; // name of the action

    igvc_msgs::map *currMap_; // current map of the environment
    cv_bridge::CvImageConstPtr currMap_cvptr_; // opencv pointer of currMap_

    geometry_msgs::PointStamped waypoint_target_; // where we're headed

    std::unique_ptr<nav_msgs::Path> path_; // path the robot is taking
    std::unique_ptr<nav_msgs::Path> trajectory_; // Smooth control trajectory generated for the path
    igvc_msgs::velocity_pair vel_; // velocities for motor

    igvc_msgs::PathExecuteFeedback feedback_; // feedback we're sending
    igvc_msgs::PathExecuteResult result_; // sent upon end of action

    ros::ServiceClient path_client_; // client to path generation service
    igvc_msgs::GeneratePath path_srv_; // service request sent to the path generation service

    ros::Subscriber pose_sub_; // subscriber to robot odometry
    ros::Subscriber map_sub_; // subscriber to map

    ros::Publisher cmd_pub_; // publish commands to motors
    ros::Publisher target_pub_; // publish current target
    ros::Publisher trajectory_pub_; // publish planned trajectory

    SmoothControl controller_; // control law generator

    double lookahead_dist_, maximum_vel_, c_space_, probability_threshold_;

private:
    static double get_distance(double x1, double y1, double x2, double y2);
    static double format_double(double val, double scale);
    bool isRouteValid(std::unique_ptr<nav_msgs::Path>& route);
    template<typename T, typename... Args>
    std::unique_ptr<T> make_unique(Args&&... args);
};

#endif //PATHEXECUTEACTION_H
