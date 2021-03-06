cmake_minimum_required(VERSION 2.8.3)
project(bambi)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-DSPLINES_DO_NOT_USE_GENERIC_CONTAINER)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    mavros_msgs
    bambi_msgs
    geodesy
    splines
)


catkin_package(
  INCLUDE_DIRS 
  LIBRARIES bambi
  CATKIN_DEPENDS mavros_msgs bambi_msgs geodesy splines
  DEPENDS
)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
   src/mission_controller/lib/statemachine.cpp
   src/lib/utilities.cpp
   src/mission_controller/lib/mcpublisher.cpp
   src/flight_controller/lib/flightcontrollernode.cpp
   src/coverage_path_planner/lib/coveragepathplannernode.cpp
   src/coverage_path_planner/lib/common.cpp
   src/coverage_path_planner/lib/advancedwavefrontsolver.cpp
   src/coverage_path_planner/lib/cellpriority.cpp
   src/coverage_path_planner/lib/distancepriority.cpp
   src/coverage_path_planner/lib/isolatedfieldpriority.cpp
   src/coverage_path_planner/lib/spline/spline/src/main/cpp/Bezier.cpp
   src/coverage_path_planner/lib/spline/spline/src/main/cpp/BSpline.cpp
   src/coverage_path_planner/lib/spline/spline/src/main/cpp/CatmullRom.cpp
   src/coverage_path_planner/lib/spline/spline/src/main/cpp/Curve.cpp
   src/coverage_path_planner/lib/spline/spline/src/main/cpp/Vector.cpp
   src/trajectory_generator/lib/trajectorygeneratornode.cpp
)

add_executable(${PROJECT_NAME}_mission_controller src/mission_controller/mission_controller.cpp)
add_executable(${PROJECT_NAME}_coverage_path_planner src/coverage_path_planner/coverage_path_planner.cpp)
add_executable(${PROJECT_NAME}_trajectory_generator src/trajectory_generator/trajectory_generator.cpp)
add_executable(${PROJECT_NAME}_flight_controller src/flight_controller/flight_controller.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"

set_target_properties(${PROJECT_NAME}_mission_controller PROPERTIES OUTPUT_NAME mission_controller PREFIX "")
set_target_properties(${PROJECT_NAME}_coverage_path_planner PROPERTIES OUTPUT_NAME coverage_path_planner PREFIX "")
set_target_properties(${PROJECT_NAME}_trajectory_generator PROPERTIES OUTPUT_NAME trajectory_generator PREFIX "")
set_target_properties(${PROJECT_NAME}_flight_controller PROPERTIES OUTPUT_NAME flight_controller PREFIX "")

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_mission_controller ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_coverage_path_planner ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_trajectory_generator ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_flight_controller ${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
   scripts/optical_cam/optical_cam
   scripts/optical_cam/settings.py
   scripts/optical_cam/camera_photo.py
   scripts/boundary_generator/boundary_generator
   scripts/kml_generator/kml_generator
   scripts/terrain_data_provider/terrain_data_provider
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bambi.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)