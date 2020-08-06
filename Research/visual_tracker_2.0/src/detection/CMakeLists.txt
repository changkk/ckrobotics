cmake_minimum_required(VERSION 2.8.3)
project(detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  piksi_rtk_msgs
  tf2
  tf2_ros
  genmsg
  cv_bridge
  image_transport
  sensor_msgs
  smach
  smach_ros
  mavros_msgs

)
#find_package( OpenCV REQUIRED )
find_package(OpenCV 3 REQUIRED
NO_MODULE # should be optional, tells CMake to use config mode
PATHS /usr/local # look here
NO_DEFAULT_PATH) # and don't look anywhere else
find_package(CUDA REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

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
 add_message_files(
   FILES
   BaselineNed.msg
#   Message2.msg
 )

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
 generate_messages(
   DEPENDENCIES
   geometry_msgs   std_msgs	piksi_rtk_msgs	mavros_msgs
)

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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES detection
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf2 tf2_ros piksi_rtk_msgs cv_bridge image_transport sensor_msgs smach smach_ros
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
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/detection.cpp
# )

set(CUDA_NVCC_FLAGS "-arch=compute_30" CACHE STRING "nvcc flags" FORCE)
set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE) 
set(LIB_TYPE STATIC)
cuda_add_library(CUDASampLib ${LIB_TYPE} src/basic.cu)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(${PROJECT_NAME}_node src/detection_node.cpp)
 add_executable(psuedo_rtk_node src/psuedo_rtk.cpp)
 add_executable(gimbal_tf2_broadcaster_node src/gimbal_tf2_broadcaster.cpp)
 add_executable(aircraft_tf2_broadcaster_node src/aircraft_tf2_broadcaster.cpp)
 add_executable(omnicam_tf2_broadcaster_node src/omnicam_tf2_broadcaster.cpp)
 add_executable(observer_aircraft_tf2_broadcaster_node src/observer_aircraft_tf2_broadcaster.cpp)
 add_executable(mavros_target_tf2_broadcaster_node src/mavros_target_tf2_broadcaster.cpp)
 add_executable(governor_node src/governor.cpp)
 add_executable(governor_wManual_node src/governor_wManual.cpp)
 add_executable(mavlink_parser_node src/mavlink_parser.cpp)

 add_executable(color_detection_node src/color_detection.cpp)
# add_executable(color_detection_KF_node src/color_detection_KF.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(aircraft_tf2_broadcaster_node ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node CUDASampLib)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}  CUDASampLib ${CUDA_LIBRARIES}
 )
 target_link_libraries(psuedo_rtk_node
   ${catkin_LIBRARIES}
 )
 target_link_libraries(gimbal_tf2_broadcaster_node
   ${catkin_LIBRARIES}
 )
 target_link_libraries(aircraft_tf2_broadcaster_node
   ${catkin_LIBRARIES}
 )
 target_link_libraries(omnicam_tf2_broadcaster_node
   ${catkin_LIBRARIES}
 )
  target_link_libraries(observer_aircraft_tf2_broadcaster_node
   ${catkin_LIBRARIES}
 )
   target_link_libraries(mavros_target_tf2_broadcaster_node
   ${catkin_LIBRARIES}
 )
 target_link_libraries(governor_node
   ${catkin_LIBRARIES}
 )
  target_link_libraries(governor_wManual_node
   ${catkin_LIBRARIES}
 )
  target_link_libraries(mavlink_parser_node
   ${catkin_LIBRARIES}
 )
  target_link_libraries(color_detection_node 
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}  CUDASampLib ${CUDA_LIBRARIES}
 )
#  target_link_libraries(color_detection_KF_node
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
catkin_install_python(PROGRAMS src/detector_starter src/yoloStarter src/ivt_state_machine_starter src/range_calc_starter
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_detection.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
