#include <centauro_ros_nav/forward_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>


// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(forward_recovery::ForwardRecovery, nav_core::RecoveryBehavior)

namespace forward_recovery
{
ForwardRecovery::ForwardRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void ForwardRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    // we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);

    acc_lim_x_ = nav_core::loadParameterWithDeprecation(blp_nh, "acc_lim_theta", "acc_lim_th", 2.7);
    max_translational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_x", "max_vel_x", 0.5);
    min_translational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "min_vel_x", "min_vel_x", 0.05);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

ForwardRecovery::~ForwardRecovery()
{
  delete world_model_;
}

void ForwardRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if (local_costmap_ == NULL)
  {
    ROS_ERROR("The costmap passed to the ForwardRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Forward recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  geometry_msgs::PoseStamped init_global_pose;
  local_costmap_->getRobotPose(init_global_pose);

  geometry_msgs::PoseStamped global_pose;
  local_costmap_->getRobotPose(global_pose);

  double dist = 0, max_dist = 0.20;

  double current_angle;
  while (n.ok() && dist < max_dist)
    {
      local_costmap_->getRobotPose(global_pose);
      double x = global_pose.pose.position.x, y = global_pose.pose.position.y;
      current_angle = tf2::getYaw(global_pose.pose.orientation);

      x += 3.0*sim_granularity_*cos(current_angle);
      y += 3.0*sim_granularity_*sin(current_angle);

      // make sure that the point is legal, if it isn't... we'll abort
      double footprint_cost = world_model_->footprintCost(x, y, current_angle, local_costmap_->getRobotFootprint(), 0.0, 0.0);
      if (footprint_cost < 0.0)
      {
        ROS_ERROR("Forward recovery can't move front because there is a potential collision. Cost: %.2f",
                  footprint_cost);
        return;
      }

      dist = sqrt(pow(init_global_pose.pose.position.x - global_pose.pose.position.x,2) +
                  pow(init_global_pose.pose.position.y - global_pose.pose.position.y,2));

      // compute the velocity that will let us stop by the time we reach the goal
      double vel = 0.15;

      // make sure that this velocity falls within the specified limits
      vel = std::min(std::max(vel, min_translational_vel_), 0.5*max_translational_vel_);

      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = vel;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;

      vel_pub.publish(cmd_vel);

      r.sleep();
    }
}
};  // namespace forward_recovery
