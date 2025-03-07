#ifndef __VALID_TARGET_SELECT__
#define __VALID_TARGET_SELECT__

#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <XmlRpcValue.h>

#include <geometry_msgs/PoseStamped.h>
#include <centauro_ros_nav/SendCandidateNavTarget.h>
#include <nav_msgs/OccupancyGrid.h>

class ValidTargetSelectorManager {

    public:
        ValidTargetSelectorManager( std::string ns = "", double rate = 50.0 );
        ~ValidTargetSelectorManager();

        void main_loop(const ros::TimerEvent& timer);

        void spin();

    private:

        // ROS Node --------------------------------------------------------------------------------
        ros::NodeHandle nh_; /* ROSE node handle */
        ros::Timer timer_; /* ROS timer */
        double period_; /* loop period value */
        double rate_; /* loop rate */
        double time_; /* time */

        ros::Publisher send_nav_target_;
        ros::Subscriber occupancy_sub_;
        ros::ServiceServer get_candidate_target_srv_; //Todo action (?)

        geometry_msgs::PoseStamped nav_target_;
        std::string occupancy_map_topic_, footprint_param_name_;
        nav_msgs::OccupancyGrid::ConstPtr occupancy_;

        std::vector<std::array<double,2>> footprint_;
        double footprint_radius_; //Overestimate

        int candidate_pos_, temp_cell_, radius_grid_, colliding_cell_;
        std::array<double, 2> colliding_point_;

        double angle_, temp_dist_, min_dist_robot_;

        //-------------------------------------------
        void initROSNode(double rate);
        void initParams();
        
        bool defineValidTarget(geometry_msgs::PoseStamped& target, geometry_msgs::Pose& robot);

        bool setCandidateTarget(centauro_ros_nav::SendCandidateNavTarget::Request  &req,
                                centauro_ros_nav::SendCandidateNavTarget::Response &res);

        void occupancyCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

        bool checkCollisionRadius(int depth, geometry_msgs::Point robot);

};


#endif
