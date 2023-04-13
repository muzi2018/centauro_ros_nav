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

