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

void StateMachine::cb_mission_trigger_received(const mavros_msgs::BambiMissionTrigger &msg)
{
  //TODO : handle different Bambi mission trigger modes other then start one
    //flo parla di switch-case che risolvono la matrice della state machine
    //ricordiamoci, nella state machine,di aspettare che il GPS abbia fatto il fix prima di lanciare il takeoff
    //sembra che mavros publichi i messaggi su /global_position/global solo dopo che il GPS abbia fatto il fix


    //TODO: get alt_offset from BambiMissionTrigger msg
    float alt_offset = 10.f;
    m_publisher.takeOff(m_altitude+alt_offset);
    ROS_INFO("--BAMBI--  Target Takeoff altitude %f", m_altitude+alt_offset);
}

void StateMachine::cb_uav_state_change(const mavros_msgs::State &msg)
{
    ROS_INFO("State update received (Mode: %s)", msg.mode.c_str());
    m_uavState = msg;
}

void StateMachine::cb_update_altitude(const sensor_msgs::NavSatFix &navSatFix)
{
    m_altitude = static_cast<float>(navSatFix.altitude);
}

