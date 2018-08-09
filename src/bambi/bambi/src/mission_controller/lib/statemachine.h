/*
 * statemachine.h
 *
 * Created: 07 2018 by flo
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
#ifndef MISSIONCONTROLLERSTATEMACHINE_H
#define MISSIONCONTROLLERSTATEMACHINE_H

#include "mcpublisher.h"

#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/BambiMissionTrigger.h>
#include <mavros_msgs/WaypointReached.h>
#include <sensor_msgs/NavSatFix.h>
#include <bambi_msgs/Field.h>
#include <bambi_msgs/Path.h>
#include <bambi_msgs/Trajectory.h>
#include <bambi_msgs/OrthoPhoto.h>
#include <std_msgs/Bool.h>
#include <map>

namespace bambi {
namespace missioncontroller {

class StateMachine
{
public:
    typedef ros::Timer (*rosTimerProviderFunction)(ros::Duration period);
  
    /**
     * The provider function of ros::Timer is used to be independent from the node and 
     * be able to eventually unit test the StateMachine class.
     * @brief StateMachine
     * @param publisher
     * @param armTimerProvider
     */
    StateMachine(const MCPublisher &publisher, rosTimerProviderFunction armTimerProvider);

    void cb_mission_trigger_received (const mavros_msgs::BambiMissionTrigger& msg);
    void cb_uav_state_change(const mavros_msgs::State& msg);
    void cb_uav_state_extended_change(const mavros_msgs::ExtendedState& msg);
    void cb_update_global_position(const sensor_msgs::NavSatFix& navSatFix);
    void cb_arming_timer(const ros::TimerEvent&);
    void cb_mission_waypoint_reached(const mavros_msgs::WaypointReached& msg);
    void cb_orthophoto_ready(const bambi_msgs::OrthoPhoto& msg);
    void cb_boundary_generated(const bambi_msgs::Field& msg);
    void cb_coverage_path_ready(const bambi_msgs::Path& msg);
    void cb_trajectory_ready(const bambi_msgs::Trajectory& msg);
    void cb_coverage_flight_reached_home(const std_msgs::Bool& msg);
    
    enum class State {
      INIT,
      READY,
      ARMING,
      TAKING_OFF,
      STARTING_PHOTO_MISSION,
      REACHING_MISSION_START_POINT,
      TAKING_ORTHO_PHOTO,
      GENERATING_BOUNDARY,
      COVERAGE_PATH_PLANNING,
      GENERATING_TRAJECTORY,
      COVERAGE_FLIGHT,
      LANDING,
      MISSION_CANCELLING_RTL
    };
    
    enum class Command {
      MISSIONTRIGGER,
      GLOBAL_POSITION_UPDATE,
      TRY_ARM_TIMER_SHOT,
      UAV_MODE_UPDATE,
      MISSION_ITEM_REACHED,
      ORTHO_PHOTO_READY,
      BOUNDARY_GENERATED,
      COVERAGE_PATH_READY,
      TRAJECTORY_READY,
      BAMBI_FOUND,
      BAMBI_SAVED,
      COVERAGE_FC_REACHED_HOME,
      LANDED_STATE_UPDATE
    };
    
private:
    void handleStateMachineCommand(Command command, const void* msg);
    void changeState(State newState);
    
    State m_state;
    MCPublisher m_publisher;
    rosTimerProviderFunction m_armTimerProviderFunction;
    
    mavros_msgs::BambiMissionTrigger m_missionTriggerStart;
    mavros_msgs::ExtendedState::_landed_state_type m_lastUavLandedState;
    mavros_msgs::State::_mode_type m_lastUavMode;
    sensor_msgs::NavSatFix m_lastGlobalPosition;
    
    static const std::map<State, const char *> stateToStringMap;
    static const std::map<Command, const char *> commandToStringMap;
};

}
}

#endif // MISSIONCONTROLLERSTATEMACHINE_H