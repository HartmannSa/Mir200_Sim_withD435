cmake_minimum_required(VERSION 3.0.2)
project(mir_vision)

set( sample_dir ${CMAKE_CURRENT_SOURCE_DIR}/src/)

find_package(catkin REQUIRED COMPONENTS
  actionlib 
  actionlib_msgs 
  geometry_msgs
  message_generation
  nav_msgs
  roslib
  roscpp
  rospy
  std_msgs 
  tf2
  tf2_ros
  tf2_geometry_msgs
  turtlesim
)
find_package(VISP REQUIRED 
   visp_core 
   visp_mbt 
   visp_io 
   visp_gui 
   visp_sensor
)
find_package(Boost REQUIRED COMPONENTS system)
# find_package(realsense2 REQUIRED)

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
add_action_files(
  DIRECTORY action
  FILES
    CamDetection.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    std_msgs
    nav_msgs
)

###################################
## catkin specific configuration ##
###################################
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mir_vision
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_runtime nav_msgs roslib roscpp rospy std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
 # ${realsense2_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mir_vision.cpp
# )

## Add cmake target dependencies of the library
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable( image_display ${sample_dir}tutorial-image-display.cpp )
# add_executable( broadcaster ${sample_dir}broadcaster_test.cpp )
# add_executable( detection ${sample_dir}detection.cpp )
# add_executable( rgbd_tracker ${sample_dir}tutorial-mb-generic-tracker-rgbd-realsense.cpp )
# add_executable( tutorial-viewer ${sample_dir}tutorial-grabber-realsense.cpp )
add_executable( learnObject ${sample_dir}LearnObject.cpp )
add_executable( detection_client ${sample_dir}ActionClient.cpp )
add_executable( detection_server ${sample_dir}ActionServer.cpp )
add_dependencies(detection_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(detection_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(image_display ${catkin_LIBRARIES} ${VISP_LIBRARIES})
# target_link_libraries(broadcaster ${catkin_LIBRARIES} )
# target_link_libraries(detection ${catkin_LIBRARIES} ${VISP_LIBRARIES})
# target_link_libraries(rgbd_tracker ${catkin_LIBRARIES} ${VISP_LIBRARIES} )
# target_link_libraries(tutorial-viewer ${catkin_LIBRARIES} ${VISP_LIBRARIES})
target_link_libraries(learnObject ${catkin_LIBRARIES} ${VISP_LIBRARIES})
target_link_libraries(detection_client ${catkin_LIBRARIES} ${VISP_LIBRARIES})
target_link_libraries(detection_server ${catkin_LIBRARIES} ${VISP_LIBRARIES})

# copy the needed Data/model files
# visp_copy_dir(detection.cpp "${CMAKE_CURRENT_SOURCE_DIR}" model)
visp_copy_dir(detection_server "${CMAKE_CURRENT_SOURCE_DIR}" model)




