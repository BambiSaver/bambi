/*
 * missioncontroller.cpp
 *
 * Created: 2018/8/6 by Florian Mahlknecht <m@florian.world>
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
#include <ros/ros.h>
#include "lib/statemachine.h"
#include "lib/mcpublisher.h"

#include <mavros_msgs/State.h>
#include <mavros_msgs/BambiMissionTrigger.h>

using namespace bambi::missioncontroller;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bambi_missioncontroller");
  ros::NodeHandle nh;
  
  MCPublisher mcpublisher(nh);
  StateMachine stateMachine(mcpublisher);
  ros::Subscriber s1 = nh.subscribe("/mavros/bambi/missiontrigger", 10,
               &StateMachine::cb_mission_trigger_received, &stateMachine);
  ros::Subscriber s2 = nh.subscribe("/mavros/state", 10,
               &StateMachine::cb_uav_state_change, &stateMachine);
  ros::Subscriber s3 = nh.subscribe("/mavros/global_position/global",10,
                                    &StateMachine::cb_update_altitude, &stateMachine);

  //ROS_INFO("Subscriber topic: %s, count: %d", s1.getTopic().c_str(), s1.getNumPublishers());
  //ROS_INFO("Subscriber topic: %s, count: %d", s2.getTopic().c_str(), s2.getNumPublishers());
  
  ROS_INFO("Mission Controller STARTUP");

  ros::spin();
  
  return 0;
}
