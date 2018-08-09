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
#include <vector>

#include <mavros_msgs/State.h>
#include <mavros_msgs/BambiMissionTrigger.h>

using namespace bambi::missioncontroller;

// used to handle timer creation callback
StateMachine* smptr;
ros::NodeHandle* nhptr;

ros::Timer rosArmTimerProviderFunction (ros::Duration period) {
  return nhptr->createTimer(period, &StateMachine::cb_arming_timer, smptr, true, false);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "bambi_missioncontroller");
  ros::NodeHandle nh;
  
  MCPublisher mcpublisher(nh);
  StateMachine stateMachine(mcpublisher, rosArmTimerProviderFunction);
  
  // no null ptr possible, because call backs will only be active after
  // ros::spin()
  nhptr = &nh;
  smptr = &stateMachine;
  
  std::vector<ros::Subscriber> subscribers;
  
  subscribers.push_back(nh.subscribe("/mavros/bambi/missiontrigger", 5,
                        &StateMachine::cb_mission_trigger_received, &stateMachine));
  subscribers.push_back(nh.subscribe("/mavros/state", 50,
                        &StateMachine::cb_uav_state_change, &stateMachine));
  subscribers.push_back(nh.subscribe("/mavros/global_position/global",500,
                        &StateMachine::cb_update_global_position, &stateMachine));
  subscribers.push_back(nh.subscribe("/mavros/extended_state",50,
                        &StateMachine::cb_uav_state_extended_change, &stateMachine));
  subscribers.push_back(nh.subscribe("/mavros/mission/reached",5,
                        &StateMachine::cb_mission_waypoint_reached, &stateMachine));
  
  subscribers.push_back(nh.subscribe("/bambi_boundary_generator/boundary",5,
                        &StateMachine::cb_boundary_generated, &stateMachine));
  subscribers.push_back(nh.subscribe("/bambi_coverage_path_planner/path",5,
                        &StateMachine::cb_coverage_path_ready, &stateMachine));
  subscribers.push_back(nh.subscribe("/bambi_trajectory_generator/trajectory",5,
                        &StateMachine::cb_trajectory_ready, &stateMachine));
  subscribers.push_back(nh.subscribe("/bambi_flight_controller/reached_home",5,
                        &StateMachine::cb_mission_waypoint_reached, &stateMachine));
  

  ROS_INFO("Mission Controller STARTUP");

  ros::spin();
  
  return 0;
}
