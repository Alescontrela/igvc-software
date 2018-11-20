#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/client/terminal_state.h>
#include <mutex>

#include <geometry_msgs/PointStamped.h>
#include <igvc_msgs/PathExecuteAction.h>
#include <igvc_msgs/map.h>
#include <cv_bridge/cv_bridge.h>

actionlib::SimpleActionClient<igvc_msgs::PathExecuteAction>* _ac;
std::mutex planning_mutex;

ros::Subscriber _waypoint_sub;

ros::Time startTime; // action start time
ros::Duration actionTimeout; // time elapsed since action dispatched
float maxActionDuration;


void waypoint_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(planning_mutex);
    // creation action definition and send goal to action server
    if(_ac->getState() == actionlib::SimpleClientGoalState::ACTIVE)
    {
        // check if active action has exceeded timeout. If so, cancel it.
        ros::Duration timeElapsed = ros::Time::now() - startTime;

        ROS_INFO_STREAM("Action State: " << _ac->getState().toString()
                        << "| Time Elapsed:  " << timeElapsed);

        if (timeElapsed > actionTimeout) {
            _ac->cancelGoal();
            ROS_INFO("Action did not finish before timeout.");
        }
    } else {
        // otherwise, if action isn't active, create new action and dispatch it
        // to the action server.

        geometry_msgs::PointStamped target(*msg);

        igvc_msgs::PathExecuteGoal goal;
        goal.waypoint = target; // get goal as waypoint
        ROS_INFO_STREAM("Constructing new goal... Current State: "
                        << _ac->getState().getText());
        _ac->sendGoal(goal);
        ROS_INFO_STREAM("Goal Sent to Action Server... Current State: "
                        << _ac->getState().getText());

        ROS_INFO_STREAM("Navigating to waypoint: ["
                                                  << goal.waypoint.point.x
                                                  << ","
                                                  << goal.waypoint.point.y
                                                  << "]");
        startTime = ros::Time::now();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pather");

    ros::NodeHandle nh;
    ros::NodeHandle actionNh; // node handler for the action client
    ros::NodeHandle pNh("~");

    // make sure all required parameters have been supplied)
    std::vector<std::string> params = {"max_action_time"};

    for (auto it = params.begin(); it != params.end(); it++)
    {
        if (!pNh.hasParam(*it)) {
            ROS_ERROR_STREAM("Pather does not contain "
                            << "required parameter: " << *it
                            << "... aborting service");
            return 0;
        }
    }

    pNh.param<float>("max_action_time", maxActionDuration, 10);
    actionTimeout = ros::Duration(maxActionDuration); // max time alloted for action completion.

    // sub to waypoint topic
    _waypoint_sub = nh.subscribe(
        "/waypoint", 1, waypoint_callback
        );

    // create the action client responsible for requesting actions that lead
    // the robot to the supplied waypoint
    _ac = new actionlib::SimpleActionClient<igvc_msgs::PathExecuteAction>(actionNh, "path_follow_server", true);

    ROS_INFO("Waiting for the action server to start.");
    _ac->waitForServer();
    ROS_INFO("Path master connected to action server.");



    ros::spin();

    return 0;



}
