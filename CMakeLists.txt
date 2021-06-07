cmake_minimum_required(VERSION 3.0.2)
project(pongrobot_perception)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  image_transport
  tf
  tf2
  pcl_ros
)

find_package(OpenCV REQUIRED)

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
#  LIBRARIES brobot_vision
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###################################
## build
###################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  include
  ${catkin_INCLUDE_DIRS}
)

# executables 
add_executable( tf_broadcaster src/tf_broadcaster.cpp )
target_link_libraries( tf_broadcaster ${catkin_LIBRARIES} )

add_executable( cup_detector_node src/cup_detector_node.cpp src/CupDetector.cpp )
target_link_libraries( cup_detector_node ${catkin_LIBRARIES} )

add_executable( game_logic_node src/game_logic_node.cpp src/GameManager.cpp )
target_link_libraries( game_logic_node ${catkin_LIBRARIES} )
