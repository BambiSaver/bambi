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


#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>

//#include <ros/service_client.h>

using namespace bambi::flight_controller;

FlightControllerNode::FlightControllerNode(const ros::NodeHandle &nodeHandle)
    : m_nodeHandle(nodeHandle)
    , m_rate(1.0)
    , m_state(State::READY) {

    m_subscriberCoverageFlightTrigger = m_nodeHandle.subscribe(
        "/bambi/mission_controller/trigger_coverage_flight", 5,
       &FlightControllerNode::cb_trigger_coverage_flight, this);

    m_subscriberHoverTrigger = m_nodeHandle.subscribe(
        "/bambi/mission_controller/trigger_hover", 5,
       &FlightControllerNode::cb_trigger_hover, this);

    m_subscriberHoveringPosition = m_nodeHandle.subscribe(
        "/bambi/mission_controller/hovering_position", 500,
       &FlightControllerNode::cb_hovering_position, this);

    m_publisherSetPosition = m_nodeHandle.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 500, false);

    m_publisherReachedHome = m_nodeHandle.advertise<std_msgs::Bool>(
        "reached_home", 5, false);

    m_subscriberVehicleState = m_nodeHandle.subscribe(
                "/mavros/state", 10,
                &FlightControllerNode::cb_uav_state_change, this);

//    m_serviceClientSetMode = m_nodeHandle.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}




void FlightControllerNode::cb_trigger_coverage_flight(const bambi_msgs::Trajectory &trajectory) {
    ROS_INFO("Flight Controller got coverage flight trigger with trajectory");
    handleStateMachineCommand(Command::NEW_TRAJECTORY_RECEIVED, &trajectory);
}

void FlightControllerNode::cb_uav_state_change(const mavros_msgs::State &msg) {
    ROS_INFO_THROTTLE(5, "Mode update (==> '%s')", msg.mode.c_str());

    if (msg.mode == "OFFBOARD") {
        handleStateMachineCommand(Command::UAV_MODE_CHANGED_TO_OFFBOARD, NULL);
    }
}

void FlightControllerNode::cb_bias_setpoints_timer(const ros::TimerEvent &) {
    handleStateMachineCommand(Command::TIME_TO_CHANGE_MODE, NULL);
}

void FlightControllerNode::cb_trigger_hover(const std_msgs::Bool &hoverOn) {
    ROS_INFO("Flight Controller got hover trigger %s", hoverOn.data ? "ON" : "OFF");
}

void FlightControllerNode::cb_hovering_position(const mavros_msgs::GlobalPositionTarget &hoveringPosition) {
    ROS_INFO("Flight Controller got hovering position update");
}

void FlightControllerNode::changeState(FlightControllerNode::State newState) {
    ROS_INFO("Changing flight controller state from %s to %s", stateToStringMap.at(m_state), stateToStringMap.at(newState));
    m_state = newState;
}

void FlightControllerNode::spin() {
    ros::AsyncSpinner spinner(2);
    spinner.start();
    while (ros::ok()) {
        switch (m_state) {
        case State::PUBLISHING_FIRST_POINTS:
        case State::WAITING_FOR_MODE_CHANGE:
        case State::REACHED_HOME: // continue to publish last setpoint
        case State::FLYING:
            //ROS_INFO("PUBLISHING FIRST POINTS (index = %d", static_cast<int>(m_index));

            mavros_msgs::PositionTarget pos;
            pos.position.x = 89.0;
            pos.position.y = 19.0;
            pos.position.z = 45.0;
            pos.velocity.x = 0;
            pos.velocity.y = 0;
            pos.velocity.z = 0;
            pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            pos.header.stamp = ros::Time::now();
            pos.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

            m_publisherSetPosition.publish(pos);
            break;
//        case State::FLYING:
            // DO NOT FLY FOR NOW

//            ++m_index;
//            if (m_index < m_trajectory->setpoints.size()) {
//                m_trajectory->setpoints[m_index].header.stamp = ros::Time::now();
//                m_publisherSetPosition.publish(m_trajectory->setpoints[m_index]);
//            } else {
//                // reached home
//                // return index to save value
//                --m_index;
//                std_msgs::Bool b;
//                b.data = true;
//                m_publisherReachedHome.publish(b);
//                changeState(State::REACHED_HOME);
//            }
//            break;
        }
        //ros::spinOnce();
        m_rate.sleep();
    }
    spinner.stop();
}

void FlightControllerNode::handleStateMachineCommand(FlightControllerNode::Command command, const void *msg) {

    m_mutex.lock();


    switch (m_state) {
    case State::READY:
        if (command == Command::NEW_TRAJECTORY_RECEIVED) {
            auto trajectory = static_cast<const bambi_msgs::Trajectory*>(msg);
            m_index = 0;
            m_rate = ros::Rate(trajectory->sample_rate);
            m_trajectory = boost::shared_ptr<bambi_msgs::Trajectory>(new bambi_msgs::Trajectory(*trajectory));
            changeState(State::PUBLISHING_FIRST_POINTS);
            m_biasSetpointTimer = m_nodeHandle.createTimer(ros::Duration(2.0), &FlightControllerNode::cb_bias_setpoints_timer, this, true, false);
            m_biasSetpointTimer.start();
        } else {
            ROS_WARN("Ignoring command %s in state READY", commandToStringMap.at(command));
        }
        break;
    case State::PUBLISHING_FIRST_POINTS:
        if (command == Command::TIME_TO_CHANGE_MODE) {

            mavros_msgs::SetMode commandSetMode;
            //commandSetMode.request.base_mode = 1; //stands for MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            commandSetMode.request.custom_mode = "OFFBOARD";


            changeState(State::WAITING_FOR_MODE_CHANGE);

            if (!ros::service::waitForService("/mavros/set_mode", 5)){
                ROS_ERROR("The service /mavros/set_mode is not avaiable");

            }


            if (!ros::service::call("/mavros/set_mode",commandSetMode)) {
                  ROS_ERROR("Set_mode service cannot send SetMode message");

            }

            if (commandSetMode.response.mode_sent == commandSetMode.request.base_mode){
                  ROS_INFO("Mode id %d succesfully sent", commandSetMode.response.mode_sent);
            }

//            // TODO wait for service, error handling ecc.
//            if (m_serviceClientSetMode.call(commandSetMode)) {
//                // sent correctly, just wait for callback
//            } else {
//                ROS_WARN("Set Mode failed, retrying");
//                changeState(State::PUBLISHING_FIRST_POINTS);
//                m_biasSetpointTimer = m_nodeHandle.createTimer(ros::Duration(2.0), &FlightControllerNode::cb_bias_setpoints_timer, this, true, false);
//                m_biasSetpointTimer.start();
//                // retrying
//            }
        } else {
            ROS_WARN("Ignoring command %s in state PUBLISHING_FIRST_POINTS", commandToStringMap.at(command));
        }
        break;
    case State::WAITING_FOR_MODE_CHANGE:
        if (command == Command::UAV_MODE_CHANGED_TO_OFFBOARD) {
            ROS_INFO("ALMOST READY TO FLY THE COVERAGE MISSION!");
            changeState(State::FLYING);
        } else {
            ROS_WARN("Ignoring command %s in state WAITING_FOR_MODE_CHANGE", commandToStringMap.at(command));
        }
        break;
    case State::FLYING:
        if (command == Command::HOVER_TRIGGER) {
            ROS_WARN("Got hover trigger, DON'T KNOW WHAT TO DO NOW [IMPLEMENT ME]");
        } else {
            ROS_WARN("Ignoring command %s in state FLYING", commandToStringMap.at(command));
        }
        break;
    case State::HOVERING:
        if (command == Command::HOVER_TRIGGER) {
            ROS_WARN("Got hover trigger, DON'T KNOW WHAT TO DO NOW [IMPLEMENT ME]");
        } else {
            ROS_WARN("Ignoring command %s in state HOVERING", commandToStringMap.at(command));
        }
        break;
    case State::REACHED_HOME:
        // TODO: implement callback from missiontrigger which says us to stop publishing setpoints
        ROS_WARN("Ignoring command %s in state REACHED_HOME", commandToStringMap.at(command));
        /*
        if (command == Command::) {

        } else {
            ROS_WARN("Ignoring command %s in state REACHED_HOME", commandToStringMap.at(command));
        }*/
        break;
    }


    m_mutex.unlock();
}



const std::map<FlightControllerNode::State, const char *>  FlightControllerNode::stateToStringMap = {

  { FlightControllerNode::State::READY, "READY" },
  { FlightControllerNode::State::TRAJECTORY_RECEIVED, "TRAJECTORY_RECEIVED" },
  { FlightControllerNode::State::PUBLISHING_FIRST_POINTS, "PUBLISHING_FIRST_POINTS" },
  { FlightControllerNode::State::WAITING_FOR_MODE_CHANGE, "WAITING_FOR_MODE_CHANGE" },
  { FlightControllerNode::State::FLYING, "FLYING" },
  { FlightControllerNode::State::HOVERING, "HOVERING" },
  { FlightControllerNode::State::REACHED_HOME, "REACHED_HOME" },
};

const std::map<FlightControllerNode::Command, const char *>  FlightControllerNode::commandToStringMap = {
  { FlightControllerNode::Command::NEW_TRAJECTORY_RECEIVED, "NEW_TRAJECTORY_RECEIVED" },
  { FlightControllerNode::Command::TIME_TO_CHANGE_MODE, "TIME_TO_CHANGE_MODE" },
  { FlightControllerNode::Command::UAV_MODE_CHANGED_TO_OFFBOARD, "UAV_MODE_CHANGED_TO_OFFBOARD" },
  { FlightControllerNode::Command::HOVER_TRIGGER, "HOVER_TRIGGER" },
};



