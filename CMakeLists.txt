cmake_minimum_required(VERSION 3.0)
project(centauro_ros_nav)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf
  tf_conversions
  eigen_conversions
  geometry_msgs
  xbot_msgs
  nav_core
  message_generation
)

add_service_files(
  FILES
  SendCandidateNavTarget.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES perception_thesis_gap
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs tf tf_conversions message_generation message_runtime nav_core
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )


add_library(ValidTargetSelectorManager src/ValidTargetSelectorManager.cpp)
add_dependencies(ValidTargetSelectorManager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ValidTargetSelectorManager ${catkin_LIBRARIES})

add_executable(ValidTargetSelectorNode src/ValidTargetSelectorNode.cpp)
target_link_libraries(ValidTargetSelectorNode ${catkin_LIBRARIES} ValidTargetSelectorManager)

add_executable(cmd_add_stamp src/cmd_add_stamp.cpp)
target_link_libraries(cmd_add_stamp ${catkin_LIBRARIES})


 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    )
