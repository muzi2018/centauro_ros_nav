#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <geometry_msgs/PoseStamped.h>

//Init: acquire foot print and map_topic name

// 1. Get request of target
// 2. Get input the map
// 3. Check neighbors in the map (based on footprint radius)
// 4. get closer obstacle --> try to sample a valid candidate in the opposite direction

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ValidTargetSelector");
    ros::NodeHandle nh_;
               
    ros::Publisher send_nav_target_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    geometry_msgs::PoseStamped nav_target_;
    std::string occupancy_map_topic_;

    ros::Rate loop_rate(20);

    while(ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();

    }
    
    return 0;
}
