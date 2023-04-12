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

add_service_files(
  FILES
  AskNavPlan.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception_thesis_gap
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs tf tf_conversions message_generation message_runtime nav_core
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_library(NavCtrlManager src/NavCtrlManager.cpp)
add_dependencies(NavCtrlManager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(NavCtrlManager ${catkin_LIBRARIES})

add_executable(NavCtrlNode src/NavCtrlNode.cpp)
target_link_libraries(NavCtrlNode ${catkin_LIBRARIES} NavCtrlManager)

add_library(backward_recovery src/backward_recovery.cpp)
add_dependencies(backward_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(backward_recovery ${catkin_LIBRARIES})

add_library(forward_recovery src/forward_recovery.cpp)
add_dependencies(forward_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(forward_recovery ${catkin_LIBRARIES})

add_executable(cmd_add_stamp src/cmd_add_stamp.cpp)
target_link_libraries(cmd_add_stamp ${catkin_LIBRARIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

## Mark cpp header files for installation
install(TARGETS backward_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(TARGETS forward_recovery
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
      )

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    )

install(FILES backward_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES forward_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


