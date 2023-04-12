#ifndef __NAV_CTRL_MANAGER__
#define __NAV_CTRL_MANAGER__

#include <ros/ros.h>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/Geometry"

//Services
#include <centauro_ros_nav/AskNavPlan.h>

#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavCtrlManager {

public:
    NavCtrlManager( std::string ns = "", double rate = 50.0 );
    ~NavCtrlManager();

    void main_loop(const ros::TimerEvent& timer);

    void spin();

private:

    // ROS Node --------------------------------------------------------------------------------
    ros::NodeHandle _nh; /* ROSE node handle */
    ros::Timer _timer; /* ROS timer */
    double _period; /* loop period value */
    double _rate; /* loop rate */
    double _time; /* time */

    ros::ServiceClient _make_plan_srv;
    ros::ServiceServer _ask_for_plan_srv;

    nav_msgs::GetPlan _planned_trajctory;
    geometry_msgs::PoseStamped _start;
    move_base_msgs::MoveBaseGoal _goal;

    //-------------------------------------------
    void initROSNode(double rate);

    void initParams();

    bool askNavPlan(centauro_ros_nav::AskNavPlan::Request  &req, centauro_ros_nav::AskNavPlan::Response &res);
};


#endif
