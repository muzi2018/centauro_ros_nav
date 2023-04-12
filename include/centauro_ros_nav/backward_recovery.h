#ifndef BACKWARD_RECOVERY_BACKWARD_RECOVERY_H
#define BACKWARD_RECOVERY_BACKWARD_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/costmap_model.h>
#include <string>

namespace backward_recovery
{

	class BackwardRecovery : public nav_core::RecoveryBehavior
	{
	public:

	  BackwardRecovery();
	   
	  void initialize(std::string name, tf2_ros::Buffer*,
		          costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

	  void runBehavior();

	  ~BackwardRecovery();

	private:
	  costmap_2d::Costmap2DROS* local_costmap_;
	  bool initialized_;
      double sim_granularity_, min_translational_vel_, max_translational_vel_, acc_lim_x_, tolerance_, frequency_;
	  base_local_planner::CostmapModel* world_model_;
	};
};
#endif  // BACKWARD_RECOVERY_BACKWARD_RECOVERY_H
