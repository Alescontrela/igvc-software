#include "PathExecuteAction.h"

PathExecuteAction::PathExecuteAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name)
{
    // callback to use when registering goal
    as_.registerGoalCallback(boost::bind(&PathExecuteAction::goalCB, this));
    // callback to use when preempting action
    as_.registerPreemptCallback(boost::bind(
            &PathExecuteAction::preemptCB, this
        ));
    cmd_pub_ = nh_.advertise<igvc_msgs::velocity_pair>("/motors", 1);
    target_pub_ = \
            nh_.advertise<geometry_msgs::PointStamped>("/target_point", 1);
    trajectory_pub_ = nh_.advertise<nav_msgs::Path>("/trajectory", 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/path", 1);


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
    pNh.param(std::string("c_space"), c_space_, 0.5);
    pNh.param(std::string("probability_threshold"), probability_threshold_, 0.7);

    as_.start();
    ROS_INFO("Action Server Started.");
}

PathExecuteAction::~PathExecuteAction(void) {
    // TODO
}

void PathExecuteAction::map_callback(const igvc_msgs::mapConstPtr& msg)
{
    currMap_ = new igvc_msgs::map(*msg);
    currMap_cvptr_ = cv_bridge::toCvShare(msg->image, msg, "mono8");
}

void PathExecuteAction::goalCB()
{
    // set our target waypoint and map
    igvc_msgs::PathExecuteGoalConstPtr goal = as_.acceptNewGoal();
    waypoint_target_ = goal->waypoint;
}

void PathExecuteAction::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void PathExecuteAction::pathExecuteCB(const nav_msgs::OdometryConstPtr& msg) {
    if (!as_.isActive() || currMap_ == nullptr) {
        // make sure the action is active and that a map has been published
        return;
    }

    load_new_path();
    load_new_trajectory(msg);

    if (path_->poses.empty() || path_->poses.size() < 2)
    {
      ROS_ERROR("Path empty. Stopping Action.");
      igvc_msgs::velocity_pair vel;
      vel.left_velocity = 0.;
      vel.right_velocity = 0.;
      cmd_pub_.publish(vel);
      path_ = nullptr;
      return;
    }

    if (trajectory_->poses.empty() || trajectory_->poses.size() < 2)
    {
      ROS_ERROR("Trajectory empty. Stopping Action.");
      igvc_msgs::velocity_pair vel;
      vel.left_velocity = 0.;
      vel.right_velocity = 0.;
      cmd_pub_.publish(vel);
      trajectory_ = nullptr;
      return;
    }

    // if (!isRouteValid(path_) || !isRouteValid(trajectory_)) {
    //     ROS_ERROR("Route Invalid. Requerying...");
    //     igvc_msgs::velocity_pair vel;
    //     vel.left_velocity = 0.;
    //     vel.right_velocity = 0.;
    //     cmd_pub_.publish(vel);
    //     path_ = nullptr;
    //     trajectory_ = nullptr;
    //     return;
    // }

    // vel_.header.stamp = ros::Time::now();
    // path_->poses.erase(path_->poses.begin());
    // trajectory_->poses.erase(trajectory_->poses.begin());

    cmd_pub_.publish(vel_);
    trajectory_pub_.publish(*trajectory_);

    double distance_left = get_distance( \
            msg->pose.pose.position.x, msg->pose.pose.position.y,
            waypoint_target_.point.x, waypoint_target_.point.y);

    ROS_INFO_STREAM("Distance Left: " << distance_left);

    feedback_.distance_left = distance_left;
    feedback_.curr_path.clear();
    for (auto it = trajectory_->poses.begin(); it != trajectory_->poses.end(); it++)
        feedback_.curr_path.push_back(*it);

    as_.publishFeedback(feedback_);

    if (distance_left < 1) {
        // TODO: improve quality of message when goal is reached
        result_.waypoint_reached = true;

        ROS_INFO("Waypoint Reached. Continuing Path Execution.");
        as_.setSucceeded(result_);
    }
}

void PathExecuteAction::load_new_path() {
    /*
    Obtains a path through the environment by making a call to the path
    planner service
    */

    // construct the service call using the initial map and send to the path generator

    // wait for topic to become available
    ros::topic::waitForMessage<igvc_msgs::map>(map_sub_.getTopic(), ros::Duration(5));
    path_srv_.request.map = *currMap_;
    path_srv_.request.waypoint = waypoint_target_;
    path_srv_.request.cspace = c_space_;
    path_srv_.request.probability_threshold = probability_threshold_;

    // call the service
    if (path_client_.call(path_srv_)) {
        ROS_ERROR("Valid Path Found...Proceeding to Navigate.");
    } else {
        ROS_ERROR("Could Not Find Valid Path...Aborting Action.");
        as_.setAborted(result_);
    }

    // TODO switch over to c++14
    path_ = make_unique<nav_msgs::Path>(path_srv_.response.path);
    path_pub_.publish(path_srv_.response.path);
}

void PathExecuteAction::load_new_trajectory(const nav_msgs::OdometryConstPtr& msg) {
    /*
    Loads a new trajectory using the most recently loaded path and the
    current odometry information.
    */

    // get relevant odometry information from the odom msg
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

    // get the current theta
    geometry_msgs::Point point1, point2;
    if (path_index == 0) {
        point1 = path_->poses[path_index].pose.position;
        point2 = path_->poses[path_index + 1].pose.position;
    } else {
        point1 = path_->poses[path_index - 1].pose.position;
        point2 = path_->poses[path_index].pose.position;
    }





    // find furthest point along trajectory that isn't further than the
    // lookahead distance
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

    // determining the target theta:
    // get i and j components of vector to target point from current point, a.k.a. line of sight (los)
    double slope_x = tar_y - cur_y;
    double slope_y = tar_y - cur_y;

    Eigen::Vector3d los(slope_x, slope_y, 0); // line of sight
    los.normalize();

    // get i and j components of target orientation vector (res_orientation)
    double distance = 0;
    geometry_msgs::Point point1, point2;
    unsigned int last_point_idx = 0;
    while (last_point_idx < path_->poses.size() - 1)
    {
        point1 = path_->poses[last_point_idx].pose.position;
        point2 = path_->poses[last_point_idx + 1].pose.position;
        double increment = get_distance(point1.x, point1.y, point2.x, point2.y);
        if (distance + increment > lookahead_dist_) {break;}

        last_point_idx++;
        distance += increment;
    }

    double pose_x = point2.x - point1.x;
    double pose_y = point2.y - point1.y;

    Eigen::Vector3d res_orientation(pose_x, pose_y, 0); // resultant orientation
    res_orientation.normalize();

    Eigen::Vector3d delta_orientation = res_orientation - los;
    tar_theta = atan2(delta_orientation[1], delta_orientation[0]);

    ros::Time time = msg->header.stamp;

    geometry_msgs::PointStamped target_point;
    target_point.header.frame_id = "/odom";
    target_point.header.stamp = time;
    target_point.point.x = tar_x;
    target_point.point.y = tar_y;
    target_pub_.publish(target_point);

    nav_msgs::Path trajectory_msg;

    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_msg.header.frame_id = "/odom";

    Eigen::Vector3d cur_pos(cur_x, cur_y, cur_theta);
    Eigen::Vector3d target(tar_x, tar_y, tar_theta);

    // get trajectory from the smooth controller_
    controller_.getTrajectory(vel_, trajectory_msg, cur_pos, target);

    trajectory_ = make_unique<nav_msgs::Path>(trajectory_msg);
}

double PathExecuteAction::get_distance(double x1, double y1, double x2, double y2) {
    /*
    Returns euclidian distance between two points.
    */
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

double PathExecuteAction::format_double(double val, double scale) {
    /*
    Formats a double using the specified scale. i.e.

    tenths     - 0.10
    hundredths - 0.01
    */
    return std::floor(val / scale + 0.5) * scale;
}

bool PathExecuteAction::isRouteValid(std::unique_ptr<nav_msgs::Path>& route) {
    /*
    Checks whether or not a route is valid in the current map.
    */

    // get relevant parameters from map
    float resolution = currMap_->resolution;
    int curr_x = currMap_->x;
    int curr_y = currMap_->y;

    double min_val, max_val;
    double sep = c_space_/resolution;

    for (auto it = route->poses.begin(); it != route->poses.end(); it++) {
        int map_height = currMap_cvptr_->image.size().height;
        int map_width = currMap_cvptr_->image.size().width;

        // gather portion of map to check based on the c_space
        int check_x_lower = std::max(static_cast<int>(std::round(it->pose.position.x/resolution + curr_x - sep)), 0);
        int check_y_lower = std::max(static_cast<int>(std::round(it->pose.position.y/resolution + curr_y - sep)), 0);
        int check_x_upper = std::min(static_cast<int>(std::round(it->pose.position.x/resolution + curr_x + sep)), map_height);
        int check_y_upper = std::min(static_cast<int>(std::round(it->pose.position.y/resolution + curr_y + sep)), map_width);

        cv::Mat subsection = \
            currMap_cvptr_->image(cv::Range(check_x_lower, check_x_upper),
                                  cv::Range(check_y_lower, check_y_upper));

        // check whether or not point along path is likely to be occupied
        cv::minMaxLoc(subsection, &min_val, &max_val);
        if (max_val > probability_threshold_ * 255) return false;
    }

    return true;


}

template<typename T, typename... Args>
std::unique_ptr<T> PathExecuteAction::make_unique(Args&&... args) {
    /*
    Same as std::make_unique in c++14
    */
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
