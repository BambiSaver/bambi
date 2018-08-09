/*
 * flightcontrollernode.cpp
 *
 * Created: 2018/8/8 by Florian Mahlknecht <m@florian.world>
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
#include "flightcontrollernode.h"


using namespace bambi::flight_controller;

FlightControllerNode::FlightControllerNode(const ros::NodeHandle &nodeHandle)
  : m_nodeHandle(nodeHandle) {
  
  m_subscriberCoverageFlightTrigger = m_nodeHandle.subscribe(
        "/bambi/missioncontroller/trigger_coverage_flight", 5,
       &FlightControllerNode::cb_trigger_coverage_flight, this);
  
  m_subscriberHoverTrigger = m_nodeHandle.subscribe(
        "/bambi/missioncontroller/trigger_hover", 5,
       &FlightControllerNode::cb_trigger_hover, this);
  
  m_subscriberHoverTrigger = m_nodeHandle.subscribe(
        "/bambi/missioncontroller/hovering_position", 500,
       &FlightControllerNode::cb_hovering_position, this);
  
  m_publisherSetPosition = m_nodeHandle.advertise<mavros_msgs::GlobalPositionTarget>(
        "/mavros/setpoint_position/global", 500, false);
  
  m_publisherReachedHome = m_nodeHandle.advertise<std_msgs::Bool>(
        "/bambi/flight_controller/reached_home", 5, false);
}

void FlightControllerNode::cb_trigger_coverage_flight(const bambi_msgs::Trajectory &trajectory) {
  ROS_INFO("Flight Controller got coverage flight trigger with trajectory");
}

void FlightControllerNode::cb_trigger_hover(const std_msgs::Bool &hoverOn) {
  ROS_INFO("Flight Controller got hover trigger %s", hoverOn.data ? "ON" : "OFF");
}

void FlightControllerNode::cb_hovering_position(const mavros_msgs::GlobalPositionTarget &hoveringPosition) {
  ROS_INFO("Flight Controller got hovering position update");
}

void FlightControllerNode::spin() {
    ros::spin();
  }


