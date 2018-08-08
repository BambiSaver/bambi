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
#include <bambi_msgs/Field.h>
#include <geographic_msgs/GeoPoint.h>

using namespace bambi::missioncontroller;

MCPublisher::MCPublisher(const ros::NodeHandle &missioncontrollerNodeHandle) :
  m_mcNodeHandle(missioncontrollerNodeHandle)
{
  m_statusTextPublisher = m_mcNodeHandle.advertise<mavros_msgs::StatusText>("/mavros/statustext/send", 500, false);
}

void MCPublisher::takeOff(int overGroundOffsetInMeters)
{
  mavros_msgs::StatusText statusText;
  statusText.severity = mavros_msgs::StatusText::INFO;
  statusText.text = "Taking off";
  
  m_statusTextPublisher.publish(statusText);
  
  bambi_msgs::Field bambiField;
  
  geographic_msgs::GeoPoint gp;
  
  
  mavros_msgs::CommandBool commandBool;
  commandBool.request.value = true;
  
  ROS_INFO("SENDING ARMING MESSAGE");
  
  if (ros::service::call("/mavros/cmd/arming", commandBool)) {
    ROS_INFO("ARMING CALL RETURNED %s", commandBool.response.success ? "successfully" : "failed");
  }
  
  
  mavros_msgs::CommandTOL commandTOL;
  commandTOL.request.latitude = std::numeric_limits<float>::quiet_NaN();
  commandTOL.request.longitude = std::numeric_limits<float>::quiet_NaN();
  commandTOL.request.altitude = 1430;
  ROS_INFO("SENDING TAKEOFF MESSAGE");
  
  if (ros::service::call("/mavros/cmd/takeoff", commandTOL)) {
    ROS_INFO("TAKE OFF CALL RETURNED %s", commandTOL.response.success ? "successfully" : "failed");
  }
  
  //"/mavros/cmd/arming"
}

