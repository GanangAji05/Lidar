cmake_minimum_required(VERSION 3.0.2)
project(filter_lidar)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  rviz
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES filter_lidar
#  CATKIN_DEPENDS pcl_ros roscpp rospy rviz sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/filter_lidar.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(publisher_node src/publisher_node.cpp)
add_executable(subscriber_node src/subscriber.cpp)
add_executable(filter_PassTrough src/filter_passThrough.cpp)
add_executable(tranfrom_pointcloud src/tranfrom_pointcloud.cpp)
add_executable(extrak_indices src/filter_extrak_Indies.cpp)
add_executable(Indices src/Indices.cpp)
add_executable(Radius_Removal src/filter_radiusremoval.cpp)
add_executable(filter_statistical_removal src/statistical_outlier_removal.cpp)
add_executable(filter_voxelgrid src/voxel_grid.cpp)
add_executable(filter_cropbox src/filter_cropbox.cpp)
add_executable(filter_median src/filter_median.cpp)
add_executable(filter_gaussian src/gaussian.cpp)
add_executable(filter_IR src/filter_cut_IR.cpp)
add_executable(morph src/filter_morphological.cpp)
add_executable(AORI src/AORI.cpp)
add_executable(smooth src/smooth.cpp)
add_executable(kalman src/kalman.cpp)
add_executable(gaus src/filter_gaus.cpp)








## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(publisher_node ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(subscriber_node ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(filter_PassTrough ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tranfrom_pointcloud ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(extrak_indices ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(Indices ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(Radius_Removal ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(filter_statistical_removal ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(filter_voxelgrid ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(filter_cropbox ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(filter_median ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(filter_gaussian ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(filter_IR ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(morph ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(AORI ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(smooth ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(kalman ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gaus ${publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})












## Specify libraries to link a library or executable target against
target_link_libraries(publisher_node
  ${catkin_LIBRARIES}
)
target_link_libraries(subscriber_node
  ${catkin_LIBRARIES}
)
target_link_libraries(filter_PassTrough
  ${catkin_LIBRARIES}
)
target_link_libraries(tranfrom_pointcloud
  ${catkin_LIBRARIES}
)
target_link_libraries(extrak_indices
  ${catkin_LIBRARIES}
)
target_link_libraries(Indices
  ${catkin_LIBRARIES}
)
target_link_libraries(Radius_Removal
  ${catkin_LIBRARIES}
)
target_link_libraries(filter_statistical_removal
  ${catkin_LIBRARIES}
)
target_link_libraries(filter_voxelgrid
  ${catkin_LIBRARIES}
)
target_link_libraries(filter_cropbox
  ${catkin_LIBRARIES}
)
target_link_libraries(filter_median
  ${catkin_LIBRARIES}
)
target_link_libraries(filter_gaussian
  ${catkin_LIBRARIES}
)
target_link_libraries(filter_IR
  ${catkin_LIBRARIES}
)
target_link_libraries(morph
  ${catkin_LIBRARIES}
)
target_link_libraries(AORI
  ${catkin_LIBRARIES}
)
target_link_libraries(smooth
  ${catkin_LIBRARIES}
)
target_link_libraries(kalman
  ${catkin_LIBRARIES}
)
target_link_libraries(gaus
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_filter_lidar.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
