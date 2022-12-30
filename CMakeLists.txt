cmake_minimum_required(VERSION 3.16)

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
  tf2_ros
  pcl_ros
)

find_package(OpenCV REQUIRED)

include(FetchContent)
include(ExternalProject)

FetchContent_Declare(
        Simple-WebSocket-Server
        GIT_REPOSITORY https://gitlab.com/eidheim/Simple-WebSocket-Server.git
)
FetchContent_MakeAvailable(Simple-WebSocket-Server)

FetchContent_Declare(json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.9.1)

FetchContent_GetProperties(json)
if(NOT json_POPULATED)
    FetchContent_Populate(json)
    add_subdirectory(${json_SOURCE_DIR} ${json_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

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

add_library(Cup src/Cup.cpp)

# executables 
add_executable( tf_broadcaster src/tf_broadcaster.cpp )
target_link_libraries( tf_broadcaster ${catkin_LIBRARIES} )

add_executable( cup_detector_node src/cup_detector_node.cpp src/CupDetector.cpp )
target_link_libraries( cup_detector_node ${catkin_LIBRARIES} )

add_executable( game_logic_node src/game_logic_node.cpp src/GameManager.cpp )
target_link_libraries( game_logic_node ${catkin_LIBRARIES} Cup)

add_executable( calibrator_node src/calibrator_node.cpp src/Calibrator.cpp )
target_link_libraries( calibrator_node ${catkin_LIBRARIES} )

add_executable( socket_telemetry_node src/socket_telemetry_node.cpp src/SocketTelemetryNode.cpp src/messagepack.cpp )
target_link_libraries( socket_telemetry_node ${catkin_LIBRARIES} )
target_link_libraries( socket_telemetry_node simple-websocket-server)
target_link_libraries( socket_telemetry_node nlohmann_json::nlohmann_json)
