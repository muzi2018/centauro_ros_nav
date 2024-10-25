cmake_minimum_required(VERSION 3.0)
project(centauro_ros_nav)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  base_local_planner
  nav_msgs
  tf
  tf_conversions
  eigen_conversions
  geometry_msgs
  xbot_msgs
  nav_core
)

#add_service_files(
#  FILES
#  AskNavPlan.srv
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception_thesis_gap
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs tf tf_conversions message_generation message_runtime nav_core
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_executable(cmd_add_stamp src/cmd_add_stamp.cpp)
target_link_libraries(cmd_add_stamp ${catkin_LIBRARIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    )
