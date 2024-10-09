#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>


#include "Eigen/Core"
#include "Eigen/Geometry"

#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"
#include <eigen_conversions/eigen_msg.h>


geometry_msgs::TwistStamped out_msg;

int zero_val = 2;

void acquire_cloud(const geometry_msgs::Twist::ConstPtr& msg)
{
    
    out_msg.twist = *msg;
    
    if(msg->linear.x == 0 && msg->linear.y == 0 && msg->angular.z == 0)
        zero_val ++;
    else
        zero_val = 0;
}


int main(int argc, char** argv)
{    
    ros::init(argc, argv, "cmd_add_stamp");
    ros::NodeHandle nh;
    
    ros::Subscriber subscriber = nh.subscribe("/centauro_plan/cmd_vel", 1, acquire_cloud);
        
    ros::Publisher transformed = nh.advertise<geometry_msgs::TwistStamped>("cartesian/car_frame/velocity_reference", 1);
    
    
    tf::StampedTransform tr;
    tf::TransformListener listener;
    ros::Time now;
    Eigen::Affine3d w_T_pelvis;
    Eigen::Vector3d linear_vel;
    
    
    //std::ofstream log_file;
    Eigen::Vector3d angular_vel, distance;
    //std::string path = ros::package::getPath("centauro_2dnav")+"/log_velocities.csv";
    //log_file.open(path);
        
    //Convert from world based to robot based !
    
    ros::Rate loop_rate(40);
    //Publish the original Point Cloud
    while(ros::ok()){

            
        //log_file << out_msg.twist.linear.x << ", " << out_msg.twist.linear.y << ", " << out_msg.twist.angular.z << "\n";
        
        if(zero_val < 2){
            out_msg.header.frame_id = "pelvis";
            transformed.publish(out_msg);
        }
        
        ros::spinOnce();
        loop_rate.sleep();

    }
    
    //log_file.close();

    return 0;
}
