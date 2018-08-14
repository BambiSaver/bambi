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
#include <mavros_msgs/HomePosition.h>

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

    m_subscriberAltitude = m_nodeHandle.subscribe(
                "/mavros/altitude", 500,
                &FlightControllerNode::cb_uav_altitude, this);

    m_subscriberHomePosition = m_nodeHandle.subscribe(
                "/mavros/home_position/home", 50,
                &FlightControllerNode::cb_uav_home_position, this);

    m_subscriberLocalPositionPose = m_nodeHandle.subscribe(
                "/mavros/local_position/pose", 500,
                &FlightControllerNode::cb_uav_local_position_pose, this);
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

void FlightControllerNode::cb_uav_altitude(const mavros_msgs::Altitude &msg) {
    if (std::isnan(msg.terrain)) {
        ROS_WARN_THROTTLE(2, "NO TERRAIN INFORMATION AVAILABLE, MAKE SURE YOU DON'T HIT THE GROUND");
    } else {
        ROS_INFO_THROTTLE(10, "Receiving altitude updates (relative to ground: %.2f)", msg.terrain);
    }
    m_lastAltitude = msg;
}

void FlightControllerNode::cb_uav_home_position(const mavros_msgs::HomePosition &msg) {
    ROS_INFO_THROTTLE(10, "Continue receiving home position updates (%.5f, %.5f)", msg.geo.latitude, msg.geo.longitude);
    m_lastHomePosition = msg;
}

void FlightControllerNode::cb_uav_local_position_pose(const geometry_msgs::PoseStamped &msg) {
    ROS_INFO_THROTTLE(10, "Contintue receiving local position updates (%.2f, %.2f, %.2f)", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    m_lastLocalPositionPose = msg;
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
    ros::AsyncSpinner spinner(4);
    spinner.start();

    while (ros::ok()) {
        switch (m_state) {
        case State::PUBLISHING_FIRST_POINTS:
        case State::WAITING_FOR_MODE_CHANGE:
            m_missionStartPositionTarget.header.stamp = ros::Time::now();
            m_publisherSetPosition.publish(m_missionStartPositionTarget);
            break;
        case State::FLYING:
            ++m_index;
            if (m_index < m_trajectory->setpoints.size()) {
                m_trajectory->setpoints[m_index].header.stamp = ros::Time::now();
                m_publisherSetPosition.publish(m_trajectory->setpoints[m_index]);
            } else {
                // reached home
                // return index to save value
                --m_index;
                std_msgs::Bool b;
                b.data = true;
                m_publisherReachedHome.publish(b);
                // TODO maybe mutex would be necessary, but in Sate::FLYING we should be fine
                // BUT eventually a TRIGGER_HOVER could get overwritten --> RACECONDITION
                // SOLUTION: ACQUIRE MUTEX ONLY HERE, AND CHECK IF STATE HASN'T BEEN CHANGED IN THE MEAN TIME
                changeState(State::REACHED_HOME);
            }
            break;
        case State::REACHED_HOME: // continue to publish last setpoint
            m_trajectory->setpoints[m_index].header.stamp = ros::Time::now();
            m_publisherSetPosition.publish(m_trajectory->setpoints[m_index]);
        }
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

            // TODO check if m_lastLocalPositionPose is valid(!)
            m_missionStartPositionTarget = createStartPositionTarget();

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
            commandSetMode.request.base_mode = 1;
            commandSetMode.request.custom_mode = "OFFBOARD";
            changeState(State::WAITING_FOR_MODE_CHANGE);

            if (!ros::service::waitForService("/mavros/set_mode", 2)
                    || !ros::service::call("/mavros/set_mode", commandSetMode)
                    || commandSetMode.response.mode_sent != commandSetMode.request.base_mode) {

                // TODO RTL after awhile --> sth like m_numberOfRetries
                ROS_ERROR("Settting mode failed, retrying... %d != %d", commandSetMode.response.mode_sent, commandSetMode.request.base_mode);
                changeState(State::PUBLISHING_FIRST_POINTS);
                m_biasSetpointTimer = m_nodeHandle.createTimer(ros::Duration(2.0), &FlightControllerNode::cb_bias_setpoints_timer, this, true, false);
                m_biasSetpointTimer.start();
            }
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




mavros_msgs::PositionTarget FlightControllerNode::createStartPositionTarget()
{
    ROS_INFO("Assigning %.2f %.2f %.2f as start position target",
             m_lastLocalPositionPose.pose.position.x,
             m_lastLocalPositionPose.pose.position.y,
             m_lastLocalPositionPose.pose.position.z);
    mavros_msgs::PositionTarget pos;
    pos.position.x = m_lastLocalPositionPose.pose.position.x;
    pos.position.y = m_lastLocalPositionPose.pose.position.y;
    pos.position.z = m_lastLocalPositionPose.pose.position.z;
    pos.velocity.x = 0;
    pos.velocity.y = 0;
    pos.velocity.z = 0;
    // align yaw with first setpoint
    pos.yaw = m_trajectory->setpoints[0].yaw;
    pos.yaw_rate = 0;
    pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pos.header.stamp = ros::Time::now();
    pos.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    if (std::isnan(m_trajectory->setpoints[0].yaw)) {
        pos.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
    }

    return pos;
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



