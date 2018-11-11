#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/PoseStamped.h>
#include <igvc_msgs/map.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <igvc_msgs/GeneratePath.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <mutex>
#include "GraphSearch.hpp"
#include "igvcsearchproblemdiscrete.h"


// debug/visualization publishers
ros::Publisher path_pub;
ros::Publisher expanded_pub;
ros::Publisher expanded_size_pub;

//TODO: restrict the scope of these variable
// search problem solver. Generates path to supplied waypoint.
IGVCSearchProblemDiscrete search_problem;
double initial_x, initial_y;
pcl::PointCloud<pcl::PointXYZ> expanded_cloud; // expanded search cloud

std::mutex planning_mutex;

void expanded_callback(const SearchLocation& location)
{
  expanded_cloud.points.push_back(
            pcl::PointXYZ(
                    (location.X - initial_x) * search_problem.Resolution,
                    (location.Y - initial_y) * search_problem.Resolution,
                    location.Theta));

  expanded_pub.publish(expanded_cloud);
  std_msgs::Int32 size_msg;
  size_msg.data = expanded_cloud.size();
  expanded_size_pub.publish(size_msg);
}

bool generate_path(igvc_msgs::GeneratePath::Request &req,
                   igvc_msgs::GeneratePath::Response &res)
{
    std::lock_guard<std::mutex> planning_lock(planning_mutex);

    // retrieve the map image from the request and store it in a pointer
    // to an OpenCV image
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(req.map.image, "mono8");

    // Construct the search problem
    search_problem.Map = cv_ptr;

    // get starting x and y coords. from the request
    search_problem.Start.X = req.map.x;
    search_problem.Start.Y = req.map.y;

    // get x_initial and y_initial from the request
    initial_x = req.map.x_initial;
    initial_y = req.map.y_initial;

    // get starting orientation from the request
    search_problem.Start.Theta =
            std::round(req.map.orientation / (M_PI / 4)) * (M_PI / 4);
    ROS_INFO_STREAM("Start position "
                    << search_problem.Start.X << ","
                    << search_problem.Start.Y
                    << " theta = " << search_problem.Start.Theta);

    search_problem.Resolution = req.map.resolution;

    // assign goal coords. to the search problem
    search_problem.Goal.X =
            std::round(req.waypoint.point.x / search_problem.Resolution) + initial_x;
    search_problem.Goal.Y =
            std::round(req.waypoint.point.y / search_problem.Resolution) + initial_y;
    ROS_INFO_STREAM("Waypoint received. grid cell = "
                    << search_problem.Goal.X << ", "
                    << search_problem.Goal.Y);


    // make sure goal isn't too far away otherwise search takes too long.
    // TODO reconsider this value once we get a new computer
    auto distance_to_goal = search_problem.Start.distTo(
            search_problem.Goal,
            search_problem.Resolution);

    // TODO get rid of magic numbers
    if (distance_to_goal == 0 || distance_to_goal > 60)
        return 0;

    // construct path msg to hold generated path
    Path<SearchLocation, SearchMove> path;
    search_problem.DistanceToGoal = distance_to_goal;

    // solve the search problem to obtain the optimal path
    path = GraphSearch::AStar(search_problem, expanded_callback);

    // construct path_msg and load it with poses -- path_msg will serve as the
    // service response
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "odom";

    for (SearchLocation loc : *(path.getStates()))
    {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = path_msg.header.stamp;
      pose.header.frame_id = path_msg.header.frame_id;
      pose.pose.position.x = (loc.X - initial_x) * search_problem.Resolution;
      pose.pose.position.y = (loc.Y - initial_y) * search_problem.Resolution;
      path_msg.poses.push_back(pose);
    }
    res.path = path_msg;
    expanded_cloud.clear();

    return 1;

}



int main(int argc, char **argv) {
    ros::init(argc, argv, "generate_path_server");

    ros::NodeHandle nh;
    ros::NodeHandle pNh("~");

    // make sure all required parameters have been supplied
    std::vector<std::string> params = {"goal_threshold",
                                       "c_space",
                                       "point_turns_enabled",
                                       "reverse_enabled",
                                       "probability_threshold"};

    for (auto it = params.begin(); it != params.end(); it++)
    {
        if (!pNh.hasParam(*it)) {
            ROS_ERROR_STREAM("Path generator does not contain "
                            << "required parameter: " << *it
                            << "... aborting service");
            return 0;
        }
    }
    // retrieve values for parameters and place them in search problem
    pNh.getParam("goal_threshold", search_problem.GoalThreshold);
    pNh.getParam("c_space", search_problem.CSpace);
    pNh.getParam("point_turns_enabled", search_problem.PointTurnsEnabled);
    pNh.getParam("reverse_enabled", search_problem.ReverseEnabled);
    pNh.getParam("probability_threshold", search_problem.ProbabilityThreshold);
    pNh.param(std::string("max_jump_size"), search_problem.MaxJumpSize, 10.0);
    pNh.param(std::string("theta_filter"), search_problem.ThetaFilter, 5.0);
    pNh.param(std::string("max_theta_change"), search_problem.MaxThetaChange, 5.0);
    pNh.param(std::string("theta_change_window"), search_problem.ThetaChangeWindow, 5.0);

    // instantiate service
    ros::ServiceServer path_generator = nh.advertiseService("generate_path", generate_path);

    expanded_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/expanded", 1);
    expanded_size_pub = nh.advertise<std_msgs::Int32>("/expanded_size", 1);
    expanded_cloud.header.frame_id = "/odom";

    ROS_INFO("Path Generation Service Spinning.");
    ros::spin();

    return 0;

}
