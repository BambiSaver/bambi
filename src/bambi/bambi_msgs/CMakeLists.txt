cmake_minimum_required(VERSION 2.8.3)
project(bambi_msgs)

add_compile_options(-std=c++11)

#find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs geographic_msgs message_generation std_msgs)

find_package(catkin
    REQUIRED COMPONENTS
    geographic_msgs
    mavros_msgs
    message_generation
    )

#include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  CoverageFlightTrigger.msg
  DynamicFlightConstraints.msg
  Field.msg
  FieldCoverageInfo.msg
  GeoPosition2D.msg
  GeoPositionWithRelativeAltitude.msg
  OrthoPhoto.msg
  Path.msg
  PathWithConstraints.msg
  Trajectory.msg
  TerrainData.msg
)

add_service_files(
    DIRECTORY srv
    FILES
    TerrainDataProvider.srv
)

#generate_messages(DEPENDENCIES geometry_msgs sensor_msgs geographic_msgs std_msgs)
generate_messages(DEPENDENCIES geographic_msgs mavros_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS #include
#  CATKIN_DEPENDS geometry_msgs sensor_msgs geographic_msgs message_runtime std_msgs)
  CATKIN_DEPENDS geographic_msgs message_runtime mavros_msgs)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
