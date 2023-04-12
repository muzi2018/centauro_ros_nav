#ifndef FORWARD_RECOVERY_FORWARD_RECOVERY_H
#define FORWARD_RECOVERY_FORWARD_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/costmap_model.h>
#include <string>

namespace forward_recovery
{

    class ForwardRecovery : public nav_core::RecoveryBehavior
	{
	public:

      ForwardRecovery();
	   
	  void initialize(std::string name, tf2_ros::Buffer*,
		          costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

	  void runBehavior();

      ~ForwardRecovery();

	private:
	  costmap_2d::Costmap2DROS* local_costmap_;
	  bool initialized_;
      double sim_granularity_, min_translational_vel_, max_translational_vel_, acc_lim_x_, tolerance_, frequency_;
	  base_local_planner::CostmapModel* world_model_;
	};
};
#endif  // FORWARD_RECOVERY_FORWARD_RECOVERY_H
