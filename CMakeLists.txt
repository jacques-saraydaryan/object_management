cmake_minimum_required(VERSION 2.8.3)
project(object_management)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
    roscpp
    rospy
    cv_bridge
    std_msgs
    sensor_msgs
    actionlib_msgs 
    actionlib
    robocup_msgs
    pepper_pose_for_nav
    darknet_gateway_srvs
    image_transport
)
## Generate services in the 'srv' folder
add_service_files(
  FILES
  ConfigureFLowSwitcherService.srv
)


## Generate actions in the 'action' folder
 add_action_files(
   FILES
   ObjectDetection.action
   LookAtObject.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   actionlib_msgs
   robocup_msgs
   # Or other packages containing msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES people_face_identification
  CATKIN_DEPENDS 
     roscpp 
     image_transport
     sensor_msgs 
     message_runtime
#  DEPENDS system_lib
)


## Build

include_directories(${catkin_INCLUDE_DIRS})
# ROS nodes
add_executable(merge_topics_to_one src/merge_topics_to_one.cpp) 
target_link_libraries(merge_topics_to_one ${catkin_LIBRARIES})
add_dependencies(merge_topics_to_one ${PROJECT_NAME}_generate_messages_cpp)
