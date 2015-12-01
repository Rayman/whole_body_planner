cmake_minimum_required(VERSION 2.8.3)
project(whole_body_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  amigo_whole_body_controller
  tue_manipulation_msgs
  arm_navigation_msgs
  geometry_msgs
  nav_msgs
  octomap_ros
  orocos_kdl
  roscpp
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)
find_package(OMPL REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES whole_body_planner
#  CATKIN_DEPENDS actionlib amigo_arm_navigation amigo_whole_body_controller arm_navigation_msgs geometry_msgs nav_msgs octomap_ros ompl orocos_kdl roscpp sensor_msgs std_msgs tf visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# show the headers in qtcreator
file(GLOB HEADERS
  RELATIVE ${PROJECT_SOURCE_DIR} "include/*.h"
  RELATIVE ${PROJECT_SOURCE_DIR} "include/*/*.h"
  RELATIVE ${PROJECT_SOURCE_DIR} "include/*/*/*.h"
)

add_library(taskspaceroadmap
  src/TaskSpaceRoadmap.cpp
  src/PlanningVisualizer.cpp
  src/VisibilityBasedValidStateSampler.cpp
  src/BoundingBoxMotionValidator.cpp
)
target_link_libraries(taskspaceroadmap
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

add_library(whole_body_planner
  src/WholeBodyPlanner.cpp
  src/Executer.cpp
  src/RobotStateInterface.cpp
  src/PlannerEmpty.cpp
  src/PlannerTopological.cpp
  src/ConnectivityGraph.cpp
  src/PlannerGlobal.cpp
  src/Simulator.cpp
  ${HEADERS}
)
target_link_libraries(whole_body_planner
  taskspaceroadmap
  ${catkin_LIBRARIES}
)

add_executable(whole_body_planner_node
  src/whole_body_planner_node.cpp
)

target_link_libraries(whole_body_planner_node
  whole_body_planner
)

# add_dependencies(whole_body_planner_node whole_body_planner_generate_messages_cpp)
