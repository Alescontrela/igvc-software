/**
* @author Alejandro Escontrela <aescontrela3@gatech.edu>
*
* Plans and executes a trajectory to a desired waypoint via the
* use of ROS actions and services.
*/

// TODO split up into class and class definition

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



class PathExecuteAction
{
public:
    PathExecuteAction(std::string name) :
        as_(nh_, name, false),
        action_name_(name)
    {
        // callback to use when registering goal
        as_.registerGoalCallback(boost::bind(&PathExecuteAction::goalCB, this));
        // callback to use when preempting action
        as_.registerPreemptCallback(boost::bind(
                &PathExecuteAction::preemptCB, this
            ));
        // motor command publisher
        cmd_pub_ = nh_.advertise<igvc_msgs::velocity_pair>("/motors", 1);
        // target position publisher
        target_pub_ = \
                nh_.advertise<geometry_msgs::PointStamped>("/target_point", 1);
        // trajectory publishers
        trajectory_pub_ = nh_.advertise<nav_msgs::Path>("/trajectory", 1);


        // subscribe to odometry -- link to callback which will realize action
        pose_sub_ = nh_.subscribe("/odometry/filtered", 1, \
                &PathExecuteAction::pathExecuteCB, this);
        // sub to map topic to get latest map
        map_sub_ = nh_.subscribe("/map", 1, \
                &PathExecuteAction::map_callback, this);

        // subscribe to path generation service
        ROS_INFO("Waiting for path generator");
        ros::service::waitForService("generate_path", ros::Duration(10.0));

        path_client_ = \
                nh_.serviceClient<igvc_msgs::GeneratePath>("generate_path");


        ROS_INFO("Action Server Connected to Path Generator.");
        // gather important starting parameters
        ros::NodeHandle pNh("~"); // private nodehandle
        pNh.param(std::string("target_v"), controller_.v, 1.0);
        pNh.param(std::string("axle_length"), controller_.axle_length, 0.52);
        pNh.param(std::string("k1"), controller_.k1, 1.0);
        pNh.param(std::string("k2"), controller_.k2, 3.0);
        pNh.param(std::string("roll_out_time"), controller_.rollOutTime, 2.0);
        pNh.param(std::string("lookahead_dist"), lookahead_dist_, 2.0);
        pNh.param(std::string("maximum_vel"), maximum_vel_, 1.6);
        as_.start();
        ROS_INFO("Action Server Started.");
    }

    ~PathExecuteAction(void)
    {
        //destructor
    }

    void goalCB()
    {
        // set our target waypoint and map
        igvc_msgs::PathExecuteGoalConstPtr goal = as_.acceptNewGoal();
        waypoint_target_ = goal->waypoint;
        load_new_path();
    }

    void load_new_path() {
        // construct the service call using the initial map and send to the path generator

        // wait for topic to become available
        ros::topic::waitForMessage<igvc_msgs::map>(map_sub_.getTopic(), ros::Duration(5));
        path_srv_.request.map = *currMap_;
        path_srv_.request.waypoint = waypoint_target_;

        // call the service
        if (path_client_.call(path_srv_)) {
            ROS_ERROR("Valid Path Found...Proceeding to Navigate.");
        } else {
            ROS_ERROR("Could Not Find Valid Path...Aborting Action.");
            as_.setAborted(result_);
        }

        // TODO switch over to c++14
        path_ = make_unique<nav_msgs::Path>(path_srv_.response.path);
    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
    }

    void map_callback(const igvc_msgs::mapConstPtr& msg)
    {
        currMap_ = new igvc_msgs::map(*msg);
    }


    void pathExecuteCB(const nav_msgs::OdometryConstPtr& msg) {
        if (!as_.isActive()) {
            // make sure the action is active
            return;
        }

        //TODO: implement to check if current trajectory interferes with map.
        // calls `load_new_path()` if true.

        if (currMap_ == nullptr) {
            // make sure a map has been published
            return;
        }

        // check if path is empty or too small
        if (path_->poses.empty() || path_->poses.size() < 2)
        {
          ROS_ERROR("Path empty. Stopping Action.");
          igvc_msgs::velocity_pair vel;
          vel.left_velocity = 0.;
          vel.right_velocity = 0.;
          cmd_pub_.publish(vel);
          return;
        }

        // get relevant odometry information (motion target) from the odom msg
        float cur_x = msg->pose.pose.position.x;
        float cur_y = msg->pose.pose.position.y;
        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
        float cur_theta = tf::getYaw(q);

        ROS_INFO_STREAM("Current: ["
                        << format_double(cur_x, .01)
                        << ","
                        << format_double(cur_y, .01)
                        << "] Target: ["
                        << format_double(waypoint_target_.point.x, .01)
                        << ","
                        << format_double(waypoint_target_.point.y, .01)
                        << "]");

        // find the closest starting position along the path
        float tar_x, tar_y, tar_theta;
        geometry_msgs::Point end = path_->poses[path_->poses.size() - 1].pose.position;
        double path_index = 0;
        double closest = std::abs(get_distance(cur_x, cur_y, path_->poses[0].pose.position.x, path_->poses[0].pose.position.y));
        double temp = std::abs(
            get_distance(cur_x, cur_y, path_->poses[path_index].pose.position.x, path_->poses[path_index].pose.position.y));
        while (path_index < path_->poses.size() && temp <= closest)
        {
          if (temp < closest)
          {
            closest = temp;
          }
          path_index++;
          temp = std::abs(
              get_distance(cur_x, cur_y, path_->poses[path_index].pose.position.x, path_->poses[path_index].pose.position.y));
        }

        if (get_distance(cur_x, cur_y, end.x, end.y) > lookahead_dist_)
        {
          double distance = 0;
          bool cont = true;
          while (cont && path_index < path_->poses.size() - 1)
          {
            geometry_msgs::Point point1, point2;
            point1 = path_->poses[path_index].pose.position;
            point2 = path_->poses[path_index + 1].pose.position;
            double increment = get_distance(point1.x, point1.y, point2.x, point2.y);
            if (distance + increment > lookahead_dist_)
            {
              cont = false;
              Eigen::Vector3d first(point1.x, point1.y, 0);
              Eigen::Vector3d second(point2.x, point2.y, 0);
              Eigen::Vector3d slope = second - first;
              slope /= increment;
              slope *= (distance - lookahead_dist_) + increment;
              slope += first;
              tar_x = slope[0];
              tar_y = slope[1];
            }
            else
            {
              path_index++;
              distance += increment;
            }
          }
        }
        else
        {
          tar_x = end.x;
          tar_y = end.y;
        }

        double yDiff = tar_y - cur_y;
        double xDiff = tar_x - cur_x;

        if (xDiff == 0)
        {
          tar_theta = yDiff > 0 ? M_PI : -M_PI;
        }
        else
        {
          tar_theta = atan2((yDiff), (xDiff));
        }

        ros::Time time = msg->header.stamp;

        geometry_msgs::PointStamped target_point;
        target_point.header.frame_id = "/odom";
        target_point.header.stamp = time;
        target_point.point.x = tar_x;
        target_point.point.y = tar_y;
        target_pub_.publish(target_point);

        igvc_msgs::velocity_pair vel;
        vel.header.stamp = time;

        nav_msgs::Path trajectory_msg;
        trajectory_msg.header.stamp = ros::Time::now();
        trajectory_msg.header.frame_id = "/odom";

        Eigen::Vector3d cur_pos(cur_x, cur_y, cur_theta);
        Eigen::Vector3d target(tar_x, tar_y, tar_theta);
        // get trajectory from the smooth controller_
        controller_.getTrajectory(vel, trajectory_msg, cur_pos, target);

        // ensure maximum wheel velocity isn't exceeded. Stop motors otherwise.
        if (vel.right_velocity > maximum_vel_ || vel.left_velocity > maximum_vel_)
        {
          ROS_ERROR_STREAM("Large velocity output stopping " << vel.right_velocity << ", " << vel.left_velocity);
          vel.right_velocity = 0;
          vel.left_velocity = 0;
        }

        double distance_left = get_distance(cur_x, cur_y, \
                waypoint_target_.point.x, waypoint_target_.point.y);

        feedback_.distance_left = distance_left;
        feedback_.curr_path.clear();
        for (auto it = trajectory_msg.poses.begin(); it != trajectory_msg.poses.end(); it++)
            feedback_.curr_path.push_back(*it);


        as_.publishFeedback(feedback_);


        if (distance_left < 1) {
            // TODO: improve quality of message when goal is reached
            result_.waypoint_reached = true;

            ROS_INFO("Waypoint Reached. Continuing Path Execution.");
            as_.setSucceeded(result_);
        }

        cmd_pub_.publish(vel);
        trajectory_pub_.publish(trajectory_msg);

    }

protected:
    ros::NodeHandle nh_; // nodehandle

    // action server
    actionlib::SimpleActionServer<igvc_msgs::PathExecuteAction> as_;
    std::string action_name_; // name of the action

    geometry_msgs::PointStamped curr_point_; // where we are
    igvc_msgs::map *currMap_; // current map of the environment

    geometry_msgs::PointStamped waypoint_target_; // where we're headed
    std::unique_ptr<nav_msgs::Path> path_; // path the robot is taking

    igvc_msgs::GeneratePath path_srv_; // service request sent to the path generation service
    igvc_msgs::PathExecuteFeedback feedback_; // feedback we're sending
    igvc_msgs::PathExecuteResult result_; // sent upon end of action

    ros::Subscriber pose_sub_; // subscriber to robot odometry
    ros::Subscriber map_sub_; // subscriber to map

    ros::ServiceClient path_client_; // client to path generation service

    ros::Publisher cmd_pub_; // publish commands to motors
    ros::Publisher target_pub_; // publish current target
    ros::Publisher trajectory_pub_; // publish planned trajectory

    SmoothControl controller_; // control law generator

    double lookahead_dist_, maximum_vel_;

private:

    static double get_distance(double x1, double y1, double x2, double y2) {
        /*
        Returns euclidian distance between two points.
        */
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }

    static double format_double(double val, double scale) {
        /*
        Formats a double using the specified scale. i.e.

        tenths     - 0.10
        hundredths - 0.01
        */
        return std::floor(val / scale + 0.5) * scale;
    }

    template<typename T, typename... Args>
    std::unique_ptr<T> make_unique(Args&&... args) {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_follow_server");
    // spin up action server
    PathExecuteAction pathExecute(ros::this_node::getName());
    while (ros::ok()) {
        ros::spinOnce();
    }


    return 0;
}
