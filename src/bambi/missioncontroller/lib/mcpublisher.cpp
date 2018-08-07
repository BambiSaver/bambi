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
}

