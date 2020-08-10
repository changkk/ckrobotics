cmake_minimum_required(VERSION 3.0.2)
project(prediction_display)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  geometry_msgs
  std_msgs
  cv_bridge
  sensor_msgs
  nav_msgs
  visualization_msgs
)

find_package(OpenCV REQUIRED)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  
)

#add_executable(stabilization src/image_stabilization.cpp src/ocam_functions.cpp)
#target_link_libraries(stabilization ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_executable(img_prediction src/image_prediction.cpp src/ocam_functions.cpp)
target_link_libraries(img_prediction ${catkin_LIBRARIES} ${OpenCV_LIBS})
