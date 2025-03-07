#include <centauro_ros_nav/ValidTargetSelectorManager.h>

int main(int argc, char** argv)
{
    ros::init (argc, argv, "ValidTargetSelector" );

    ValidTargetSelectorManager manager("valid_target_selector");
	manager.spin();

	return 0;
}
