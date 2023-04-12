#include <ros/ros.h>
#include <iostream>
#include <centauro_ros_nav/NavCtrlManager.h>

int main(int argc, char **argv)
{
    ros::init (argc, argv, "NavCtrlManager" );

    NavCtrlManager manager("nav_ctrl_node");
    manager.spin();

    return 0;
}
