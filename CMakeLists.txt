cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

find_package(octomap 1.5 REQUIRED)
include_directories(BEFORE ${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARY_DIRS}/liboctomap.so ${OCTOMAP_LIBRARY_DIRS}/liboctomath.so)
MESSAGE(STATUS "OctoMap paths:" ${OCTOMAP_INCLUDE_DIRS} " " ${OCTOMAP_LIBRARY_DIRS})

file(GLOB_RECURSE HEADER_FILES include/*.h)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OMPL REQUIRED ompl)
include_directories(${OMPL_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})
#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(taskspaceroadmap
                    src/TaskSpaceRoadmap.cpp
                    src/PlanningVisualizer.cpp
                    src/VisibilityBasedValidStateSampler.cpp
                    src/BoundingBoxMotionValidator.cpp
                    )
target_link_libraries(taskspaceroadmap ${OMPL_LIBRARIES})

rosbuild_add_library(wholebodyplanner
                    src/WholeBodyPlanner.cpp
                    src/Executer.cpp
                    src/RobotStateInterface.cpp
                    src/PlannerEmpty.cpp
                    src/PlannerTopological.cpp
                    src/ConnectivityGraph.cpp
                    src/PlannerGlobal.cpp
                    src/Simulator.cpp
                    )
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
target_link_libraries(wholebodyplanner taskspaceroadmap)
rosbuild_add_executable(whole_body_planner src/whole_body_planner_node.cpp ${HEADER_FILES})
target_link_libraries(whole_body_planner wholebodyplanner)
