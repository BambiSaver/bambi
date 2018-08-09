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
#include <sensor_msgs/NavSatStatus.h>


using namespace bambi::missioncontroller;


StateMachine::StateMachine(const MCPublisher &publisher) :
  m_state(State::INIT),
  m_publisher(publisher)
{
  
}

void StateMachine::cb_mission_trigger_received(const mavros_msgs::BambiMissionTrigger &msg)
{
  handleStateMachineCommand(Command::MISSIONTRIGGER, &msg);
}

//void StateMachine::cb_uav_state_change(const mavros_msgs::State &msg)
//{
//  ROS_INFO("State update received (Mode: %s)", msg.mode.c_str());
//  m_uavState = msg;
//}

void StateMachine::cb_update_global_position(const sensor_msgs::NavSatFix &navSatFix)
{
  // save in any case
  m_lastGlobalPosition = navSatFix;
  handleStateMachineCommand(Command::GLOBAL_POSITION_UPDATE, &navSatFix);
}

void StateMachine::handleStateMachineCommand(StateMachine::Command command, const void *msg) {
  switch (m_state) {
  case State::INIT:
    switch (command) {
    case Command::MISSIONTRIGGER:
      ROS_WARN("IGNORING MISSIONTRIGGER COMMAND, NOT READY YET");
      break;
    case Command::GLOBAL_POSITION_UPDATE: {
      auto navSatFix = (sensor_msgs::NavSatFix*)msg;
      if (navSatFix->status.status == sensor_msgs::NavSatStatus::STATUS_FIX) {
        ROS_INFO("GPS fix received, ready now");
        changeState(State::READY);
      } else {
        ROS_DEBUG("GPS fix received, but status is not STATUS_FIX, so waiting for next to change to READY");
      }
      break;
    }
    default:
      ROS_WARN("Ignoring command %s in state INIT", commandToStringHelper(command));
      break;
    }
  case State::READY:
    switch (command) {
    case Command::MISSIONTRIGGER: {
      
      auto missionTriggerMsg = (mavros_msgs::BambiMissionTrigger*)msg;
      
      if (missionTriggerMsg->startStop) {
        //TODO: get alt_offset from BambiMissionTrigger msg
        float alt_offset = 10.f;
        float altitude = static_cast<float>(m_lastGlobalPosition.altitude);
        ROS_INFO("MISSIONTRIGGER received, sending take off command");
        m_publisher.takeOff(altitude+alt_offset);
        ROS_INFO("--BAMBI--  Target Takeoff altitude %f", altitude+alt_offset);
      } else {
        ROS_WARN("MISSIONTRIGGER received: Mission not started yet, cannot STOP");
      }
      break;
    }
    case Command::GLOBAL_POSITION_UPDATE:
      // silently ignore, because position update is saved in handler, but not used in 
      break;
    default:
      ROS_INFO("Ignoring command %s in state READY", commandToStringHelper(command));
    }
    break;
  
  }
}



void StateMachine::changeState(StateMachine::State newState)
{
  ROS_DEBUG("Changing state from %s to %s", stateToStringHelper(m_state), stateToStringHelper(newState));
  m_state = newState;
}

const char *StateMachine::commandToStringHelper(StateMachine::Command command)
{
  switch (command) {
  case StateMachine::Command::MISSIONTRIGGER:
    return "MISSIONTRIGGER";
  default:
    return "## COMMAND NOT MAPPED ##";
  }
}

const char *StateMachine::stateToStringHelper(StateMachine::State state)
{
  switch (state) {
  case StateMachine::State::INIT:
    return "INIT";
  default:
    return "## STATE NOT MAPPED ##";
  }
}


