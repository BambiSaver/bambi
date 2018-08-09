/*
 * missioncontrollerstatemachine.h
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

//#include <mavros_msgs/State.h>
#include <mavros_msgs/BambiMissionTrigger.h>
#include <sensor_msgs/NavSatFix.h>

namespace bambi {
namespace missioncontroller {

class StateMachine
{
public:
    StateMachine(const MCPublisher &publisher);

    void cb_mission_trigger_received (const mavros_msgs::BambiMissionTrigger& msg);
//    void cb_uav_state_change(const mavros_msgs::State& msg);
    void cb_update_global_position(const sensor_msgs::NavSatFix& navSatFix);
    
    
    enum class State {
      INIT,
      READY,
      TAKE_OFF_SENT,
      TAKE_OFF_POSITION_REACHED,
      GOING_TO_MISSION_BASE_POINT,
      MISSION_BASE_POINT_REACHED,
      SHUTTER_TRIGGERED,
      ORTHO_PHOTO_RECEIVED,
    };
    
    enum class Command {
      MISSIONTRIGGER,
      GLOBAL_POSITION_UPDATE
    };
    
private:
    void handleStateMachineCommand(Command command, const void* msg);
    void changeState(State newState);
    
    State m_state;
    MCPublisher m_publisher;
    sensor_msgs::NavSatFix m_lastGlobalPosition;
    
    
    static const char *commandToStringHelper(StateMachine::Command command);
    static const char *stateToStringHelper(StateMachine::State state);
};

}
}

#endif // MISSIONCONTROLLERSTATEMACHINE_H