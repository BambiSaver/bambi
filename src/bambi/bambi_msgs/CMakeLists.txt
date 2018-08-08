cmake_minimum_required(VERSION 2.8.3)
project(bambi_msgs)

add_compile_options(-std=c++11)

#find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs geographic_msgs message_generation std_msgs)

find_package(catkin REQUIRED COMPONENTS geographic_msgs message_generation)

#include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  BambiField.msg
)

#add_service_files(
#)

#generate_messages(DEPENDENCIES geometry_msgs sensor_msgs geographic_msgs std_msgs)
generate_messages(DEPENDENCIES geographic_msgs)

catkin_package(
  INCLUDE_DIRS #include
#  CATKIN_DEPENDS geometry_msgs sensor_msgs geographic_msgs message_runtime std_msgs)
  CATKIN_DEPENDS geographic_msgs message_runtime)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
