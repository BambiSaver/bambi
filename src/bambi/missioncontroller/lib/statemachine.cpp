/*
 * missioncontrollerstatemachine.cpp
 *
 * Created: 07 2018 by Florian Mahlknecht <m@florian.world>
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
#include "statemachine.h"

#include <mavros_msgs/BambiMissionTrigger.h>


using namespace bambi::missioncontroller;


StateMachine::StateMachine(const MCPublisher &publisher) :
  m_state(State::READY),
  m_publisher(publisher)
{
  
}

void StateMachine::missionTriggerReceived(const mavros_msgs::BambiMissionTrigger &msg)
{
  m_publisher.takeOff(5);
}

void StateMachine::uavStateChange(const mavros_msgs::State &msg)
{
  ROS_INFO("State update received (Mode: %s)", msg.mode.c_str());
  m_uavState = msg;
}

