cmake_minimum_required(VERSION 2.8.3)
project(localizer)
set(CMAKE_CXX_FLAGS "-std=c++11") # Enable c++ 11
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  tf
  nav_msgs
  sensor_msgs
  message_filters
  image_transport
  clog_msgs
)
find_package( OpenCV 3.2 REQUIRED )

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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
#add_message_files(
#   FILES
#   ScanPose.msg
#)

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
#generate_messages(
 #  DEPENDENCIES
 #  std_msgs
 #  sensor_msgs
 #  nav_msgs
 #)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES localizer
  CATKIN_DEPENDS roscpp std_msgs clog_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_library(CLOG_CORE clog_core lib/)

## Declare a C++ library
# add_library(localizer
#   src/${PROJECT_NAME}/localizer.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(localizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(localizer_node src/localizer_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(localizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(localizer_node
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
# install(TARGETS localizer localizer_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_localizer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(rtk_simulator_node src/rtk_simulator_node.cpp)
target_link_libraries(rtk_simulator_node ${catkin_LIBRARIES}  ${OpenCV_LIBS})

add_executable(image_tf_node src/image_tf_node.cpp)
target_link_libraries(image_tf_node ${catkin_LIBRARIES}  ${OpenCV_LIBS})

add_executable(uav_localiser_node src/clog_localiser.cpp)
add_dependencies(uav_localiser_node clog_msgs_generate_messages_cpp)
add_dependencies(uav_localiser_node clog_core)
add_dependencies(uav_localiser_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(uav_localiser_node ${catkin_LIBRARIES} ${CLOG_CORE} 
 ${OpenCV_LIBS})

add_executable(display_node src/display_node.cpp)
target_link_libraries(display_node ${catkin_LIBRARIES}  ${OpenCV_LIBS})

add_executable(debug_node src/debug_node.cpp)
target_link_libraries(debug_node ${catkin_LIBRARIES}  ${OpenCV_LIBS})

add_executable(pose_sync_node src/pose_sync_node.cpp)
target_link_libraries(pose_sync_node ${catkin_LIBRARIES})

add_executable(baselink_odom_pub src/baselink_odom_pub_hack.cpp)
target_link_libraries(baselink_odom_pub ${catkin_LIBRARIES})

add_executable(fisheye_correction_node src/fisheye_correction_node.cpp src/fisheye_corrector.cpp src/ocam_functions.cpp)
target_link_libraries(fisheye_correction_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

