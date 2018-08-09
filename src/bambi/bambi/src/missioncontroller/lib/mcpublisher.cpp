/*
 * publisher.cpp
 *
 * Created: 2018/08/07 by Florian Mahlknecht <m@florian.world>
 *
 * Copyright 2018 Michael Rimondi and Florian Mahlknecht
 *
 *
 * This file is part of BAMBI.
 * BAMBI is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BAMBI is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BAMBI. If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include "mcpublisher.h"
#include <mavros_msgs/StatusText.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <bambi_msgs/Field.h>
#include <bambi_msgs/OrthoPhoto.h>
#include <bambi_msgs/FieldCoverageInfo.h>
#include <bambi_msgs/PathWithConstraints.h>
#include <bambi_msgs/Trajectory.h>
#include <geographic_msgs/GeoPoint.h>
#include <std_msgs/Bool.h>

using namespace bambi::missioncontroller;

MCPublisher::MCPublisher(const ros::NodeHandle &missioncontrollerNodeHandle)
      : m_mcNodeHandle(missioncontrollerNodeHandle) {
    m_statusTextPublisher = m_mcNodeHandle.advertise<mavros_msgs::StatusText>("/mavros/statustext/send", 500, false);

    m_triggerShutterPublisher = m_mcNodeHandle.advertise<std_msgs::Bool>("~trigger_shutter", 5, false);
    m_triggerBoundaryGenerationPublisher = m_mcNodeHandle.advertise<bambi_msgs::OrthoPhoto>("~trigger_boundary", 5, false);
    m_triggerPathGenerationPublisher = m_mcNodeHandle.advertise<bambi_msgs::FieldCoverageInfo>("~trigger_path_generation", 5, false);
    m_triggerTrajectoryGenerationPublisher = m_mcNodeHandle.advertise<bambi_msgs::PathWithConstraints>("~trigger_trajectory_generation", 5, false);
    m_triggerCoverageFlightPublisher = m_mcNodeHandle.advertise<std_msgs::Bool>("~trigger_coverage_flight", 5, false);
    m_triggerHoverPublisher = m_mcNodeHandle.advertise<std_msgs::Bool>("trigger_hover", 5, false);
    m_hoverPositionPublisher = m_mcNodeHandle.advertise<mavros_msgs::PositionTarget>("~hovering_position", 500, false);
}

bool MCPublisher::arm()
{
    mavros_msgs::CommandBool commandBool;
    commandBool.request.value = true;

    ROS_INFO("SENDING ARMING MESSAGE");


    if (!ros::service::waitForService("/mavros/cmd/arming", 5)){
        ROS_ERROR("The service /mavros/cmd/arming is not avaiable");
        return false;
    }
    if (!ros::service::call("/mavros/cmd/arming", commandBool)) {

        ROS_ERROR("Arm service cannot send arming message");
        return false;
    }
    if (commandBool.response.success){
        ROS_INFO("Arm request accepted");
        return true;
    }
  return false;
}


bool MCPublisher::takeOff(float takeoffAltitudeGlobal)
{

  //####Probably it is better to have a method which send status texts###
  mavros_msgs::StatusText statusText;
  statusText.severity = mavros_msgs::StatusText::INFO;
  statusText.text = "Taking off";
  m_statusTextPublisher.publish(statusText);

  

  
  
  mavros_msgs::CommandTOL commandTOL;
  commandTOL.request.latitude = std::numeric_limits<float>::quiet_NaN();
  commandTOL.request.longitude = std::numeric_limits<float>::quiet_NaN();
  commandTOL.request.altitude = takeoffAltitudeGlobal;
  ROS_DEBUG("SENDING TAKEOFF MESSAGE");
  
  if (!ros::service::waitForService("/mavros/cmd/takeoff", 5)){
      ROS_ERROR("The service /mavros/cmd/takeoff is not avaiable");
      return false;
  }
  if (!ros::service::call("/mavros/cmd/takeoff",commandTOL)) {
    ROS_ERROR("Takeoff service cannot send takeoff message");
    return false;
  }
  if (commandTOL.response.success){
      ROS_INFO("Take off request accepted");
      return true;
  }
return false;

}

