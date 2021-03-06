/*
 * flightcontrollernode.h
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
#ifndef FLIGHTCONTROLLERNODE_H
#define FLIGHTCONTROLLERNODE_H

#include <mutex>

#include <ros/ros.h>
#include <ros/timer.h>
#include <std_msgs/Bool.h>
#include <boost/shared_ptr.hpp>
#include <bambi_msgs/CoverageFlightTrigger.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/HomePosition.h>
#include <geometry_msgs/PoseStamped.h>

namespace bambi {
namespace flight_controller {

class FlightControllerNode
{
public:
    FlightControllerNode(const ros::NodeHandle& nodeHandle);

    void spin();


    enum class State {
        READY,
        PUBLISHING_FIRST_POINTS, // wait a few seconds before changing mode
        WAITING_FOR_MODE_CHANGE,
        FLYING,
        HOVERING,
        REACHED_HOME
    };

    enum class Command {
        COVERAGE_FLIGHT_TRIGGER_RECEIVED,
        TIME_TO_CHANGE_MODE,
        UAV_MODE_CHANGED_TO_OFFBOARD,
        HOVER_TRIGGER
    };
  
private:
  
    void cb_trigger_coverage_flight(const bambi_msgs::CoverageFlightTrigger& trajectory);
    void cb_trigger_hover(const std_msgs::Bool& hoverOn);
    void cb_hovering_position(const mavros_msgs::GlobalPositionTarget& hoveringPosition);
    void cb_uav_state_change(const mavros_msgs::State &msg);
    void cb_uav_altitude(const mavros_msgs::Altitude &msg);
    void cb_uav_home_position(const mavros_msgs::HomePosition &msg);
    void cb_uav_local_position_pose(const geometry_msgs::PoseStamped &msg);
    void cb_bias_setpoints_timer(const ros::TimerEvent&);

    void changeState(FlightControllerNode::State newState);

    void handleStateMachineCommand(Command command, const void* msg);



    ros::NodeHandle m_nodeHandle;
    ros::Publisher m_publisherSetPosition;
    ros::Publisher m_publisherReachedHome;
    ros::Subscriber m_subscriberCoverageFlightTrigger;
    ros::Subscriber m_subscriberHoverTrigger;
    ros::Subscriber m_subscriberHoveringPosition;
    ros::Subscriber m_subscriberVehicleState;
    ros::Subscriber m_subscriberAltitude;
    ros::Subscriber m_subscriberHomePosition;
    ros::Subscriber m_subscriberLocalPositionPose;
    ros::Timer m_biasSetpointTimer;

    std::mutex m_mutex;

    State m_state;
    boost::shared_ptr<bambi_msgs::Trajectory> m_trajectory;
    size_t m_index;
    ros::Rate m_rate;
    mavros_msgs::Altitude m_lastAltitude;
    mavros_msgs::HomePosition m_lastHomePosition;
    geometry_msgs::PoseStamped m_lastLocalPositionPose;
    mavros_msgs::PositionTarget m_missionStartPositionTarget;



    static const std::map<State, const char *> stateToStringMap;
    static const std::map<Command, const char *> commandToStringMap;
    mavros_msgs::PositionTarget createStartPositionTarget();
};

}
}


#endif // FLIGHTCONTROLLERNODE_H