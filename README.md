# Packages needed:
- hhcm_perception [in de_luca_perception_navigation branch: robot_2023]
- centauro_ros_nav
- base_estimation

# Dependencies
- move_base
- ros_nav

# Centauro - Real

Start the robot:
- [embedded] ecat_master
- [embedded] xbot2-core --hw ec_pos
- homing
- move velodyne to -0.6

Localization:
- [control] launch base_estimation --> publishing tf, updating pelvis frame wrt a fixed frame

Sensors:
- [control] source sensors_ws/devel/setup.bash
- [control] mon launch centauro_sensors sensor_controlPC.launch use_lateral_t265_rx:=false
- [vision] mon launch hhcm_perception camera_launcher.launch
- [vision] mon launch hhcm_perception filtering.launch

Navigation:
- [vision] mon launch centauro_ros_nav centauro_nav.launch empty_map:=true

# Centauro Navigation [Simulation]

- roslaunch hhcm_perception centauro_world_sim.launch
- xbot2-core --simtime
- homing
- move velodyne to -0.45
- mon launch hhcm_perception filtering.launch base_estim:=false
- cartesio --> car_frame in velocity mode


- mon launch centauro_ros_nav centauro_nav.launch empty_map:=true
- Check if the inflation radius is correct
- Send target with: /move_base_simple/goal or from rviz

