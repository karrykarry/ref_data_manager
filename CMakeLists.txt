cmake_minimum_required(VERSION 2.8.3)
project(ref_data_manager)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV 3 REQUIRED)
find_package(PCL 1.8.0 REQUIRED)
find_package(OpenMP)
if(OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

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
#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs#   visualization_msgs
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
  INCLUDE_DIRS include
#  LIBRARIES ref_data_manager
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include/ref_data_manager)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ref_data_manager.cpp
# )

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ref_data_manager_node.cpp)


add_executable(pr2pose node/pr2pose_node.cpp src/pr2pose.cpp)
target_link_libraries(pr2pose
	${catkin_LIBRARIES}
)


add_executable(pr2savepc src/pr2savepc.cpp)
target_link_libraries(pr2savepc
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(trans_pc src/trans_pc.cpp)
target_link_libraries(trans_pc
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(tf2odom src/tf2odom.cpp)
target_link_libraries(tf2odom
	${catkin_LIBRARIES}
)


add_executable(itst2data_pub node/itst2data_pub_node.cpp src/itst2data_pub.cpp)
target_link_libraries(itst2data_pub
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

add_executable(change_frame_pc src/change_frame_pc.cpp)
target_link_libraries(change_frame_pc
	${catkin_LIBRARIES}
)


add_executable(test_pc_save src/test/test_pc_save.cpp)
target_link_libraries(test_pc_save
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(test_pc_run node/test_pc_run_node.cpp src/test/test_pc_run.cpp)
target_link_libraries(test_pc_run
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(test_pc_only_save src/test/test_pc_only_save.cpp)
target_link_libraries(test_pc_only_save
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(test_pc_only_run src/test/test_pc_only_run.cpp)
target_link_libraries(test_pc_only_run
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)


add_executable(raycasting node/raycasting_node.cpp src/raycasting.cpp)
target_link_libraries(raycasting
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)


add_executable(pub_pcd src/test/pub_pcd.cpp)
target_link_libraries(pub_pcd
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)


add_executable(raw_pc_run src/test/raw_pc_run.cpp)
target_link_libraries(raw_pc_run
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)


add_executable(kidnapping_checker src/test/kidnapping_checker.cpp)
target_link_libraries(kidnapping_checker
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)


add_executable(rm_groundbynormal_z src/rm_groundbynormal_z.cpp)
target_link_libraries(rm_groundbynormal_z
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)


add_executable(ndt2gt src/test/ndt2gt.cpp)
target_link_libraries(ndt2gt
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ref_data_manager.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
