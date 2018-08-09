/*
 * statemachine.cpp
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
#include "../../lib/utilities.h"

#include <sensor_msgs/NavSatStatus.h>

using namespace bambi::missioncontroller;


StateMachine::StateMachine(const MCPublisher &publisher, rosTimerProviderFunction armTimerProvider) :
    m_state(State::INIT),
    m_publisher(publisher),
    m_lastUavLandedState(mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED),
    m_armTimerProviderFunction(armTimerProvider) {
}


/*
 * 
 * CALLBACKS
 * 
 */

void StateMachine::cb_mission_trigger_received(const mavros_msgs::BambiMissionTrigger &msg) {
  handleStateMachineCommand(Command::MISSIONTRIGGER, &msg);
}
void StateMachine::cb_uav_state_change(const mavros_msgs::State &msg) {
  // ONLY copy mode if it is new
  // m_lastUavMode will be empty the first time
  if (m_lastUavMode != msg.mode) {
    ROS_INFO("Mode update received ('%s' ==> '%s')", m_lastUavMode.c_str(), msg.mode.c_str());
    m_lastUavMode = msg.mode;
    handleStateMachineCommand(Command::UAV_MODE_UPDATE, (void*)&msg);
  }
}
void StateMachine::cb_uav_state_extended_change(const mavros_msgs::ExtendedState &msg) {
  if (m_lastUavLandedState != msg.landed_state) {
    ROS_INFO("Landed State update received ('%s' ==> '%s')",
             Utilities::landedStateToStringMap.at(m_lastUavLandedState),
             Utilities::landedStateToStringMap.at(msg.landed_state));
    m_lastUavLandedState = msg.landed_state;
    handleStateMachineCommand(Command::LANDED_STATE_UPDATE, (void*)&msg);
  }
}
void StateMachine::cb_update_global_position(const sensor_msgs::NavSatFix &navSatFix) {
  // save in any case
  m_lastGlobalPosition = navSatFix;
  handleStateMachineCommand(Command::GLOBAL_POSITION_UPDATE, &navSatFix);
}
void StateMachine::cb_arming_timer(const ros::TimerEvent &) {
  handleStateMachineCommand(Command::TRY_ARM_TIMER_SHOT, NULL);
}
void StateMachine::cb_mission_waypoint_reached(const mavros_msgs::WaypointReached &msg) {
  handleStateMachineCommand(Command::MISSION_ITEM_REACHED, (void*)&msg);
}
void StateMachine::cb_orthophoto_ready(const bambi_msgs::OrthoPhoto &msg) {
  handleStateMachineCommand(Command::ORTHO_PHOTO_READY, (void*)&msg);
}
void StateMachine::cb_boundary_generated(const bambi_msgs::Field &msg) {
  handleStateMachineCommand(Command::BOUNDARY_GENERATED, (void*)&msg);
}
void StateMachine::cb_coverage_path_ready(const bambi_msgs::Path &msg) {
  handleStateMachineCommand(Command::COVERAGE_PATH_READY, (void*)&msg);
}
void StateMachine::cb_trajectory_ready(const bambi_msgs::Trajectory &msg) {
  handleStateMachineCommand(Command::TRAJECTORY_READY, (void*)&msg);
}
void StateMachine::cb_coverage_flight_reached_home(const std_msgs::Bool &msg) {
  handleStateMachineCommand(Command::COVERAGE_FC_REACHED_HOME, (void*)&msg);
}




/*
 * 
 * STATE MACHINE HANDLER
 * 
 */

void StateMachine::handleStateMachineCommand(StateMachine::Command command, const void *msg) {
    switch (m_state) {
    case State::INIT:
        if (command == Command::MISSIONTRIGGER) {
            ROS_WARN("IGNORING MISSIONTRIGGER COMMAND, NOT READY YET");
            break;
        } else if (command == Command::GLOBAL_POSITION_UPDATE) {
            auto navSatFix = (sensor_msgs::NavSatFix*)msg;
            if (navSatFix->status.status == sensor_msgs::NavSatStatus::STATUS_FIX) {
                ROS_INFO("GPS fix received, ready now");
                changeState(State::READY);
            } else {
                ROS_DEBUG("GPS fix received, but status is not STATUS_FIX, so waiting for next to change to READY");
            }
        } else {
            ROS_WARN("Ignoring command %s in state INIT", commandToStringMap.at(command));
            break;
        }
        break;
    case State::READY:
        if (command == Command::MISSIONTRIGGER) {
            auto missionTriggerMsg = (mavros_msgs::BambiMissionTrigger*)msg;


            if (missionTriggerMsg->startStop) {
                //TODO: get alt_offset from BambiMissionTrigger msg
                //float alt_offset = 10.f;
                m_missionTriggerStart = *missionTriggerMsg;
                //float altitude = static_cast<float>(m_lastGlobalPosition.altitude);
                ROS_INFO("MISSIONTRIGGER received, sending ARM COMMAND");
                
                changeState(State::ARMING);
                
                // create and trigger arm timer
                //PER FLO: conviene metterlo private o ricrearlo ogni volta?
                // mi ricordo che si potrebbe anche mettere nel metodo
                //e fare in modo che non venga distrutto ad ogni chiamata
                ros::Timer timer = m_armTimerProviderFunction(ros::Duration(2.));
                timer.start();
                        
                //m_publisher.takeOff(altitude+alt_offset);
                //ROS_INFO("--BAMBI--  Target Takeoff altitude %f", altitude+alt_offset);
            } else {
                ROS_WARN("MISSIONTRIGGER received: Mission not started yet, cannot STOP");
            }
        } else if (command == Command::GLOBAL_POSITION_UPDATE) {
              // silently ignore, because position update is saved in handler, but not used in 
        } else {
            ROS_WARN("Ignoring command %s in state READY", commandToStringMap.at(command));
        }
        break;
    case State::ARMING:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
       } else if (command == Command::TRY_ARM_TIMER_SHOT) {
           if (m_publisher.arm()){
               ROS_INFO("Copter ARMED, sending takeoff message");
               float globalAltitudeTO = static_cast<float>(m_lastGlobalPosition.altitude) + m_missionTriggerStart.altitude;
                if(m_publisher.takeOff(globalAltitudeTO)){
                    changeState(State::TAKING_OFF);
                }else{
                    //### FORSE SERVE IL DISARM ###
                    ROS_WARN("TAKING OFF FAILED, going back to READY state");
                }

            }
           else{
               //TODO: set a counter which stop try arming after 5 times
               // create and trigger arm timer
               ros::Timer timer = m_armTimerProviderFunction(ros::Duration(2.));
               timer.start();
           }

       } else {
            ROS_WARN("Ignoring command %s in state ARMING", commandToStringMap.at(command));
        }
        break;
    case State::TAKING_OFF:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else {
            ROS_WARN("Ignoring command %s in state TAKING_OFF", commandToStringMap.at(command));
        }
        break;
    case State::STARTING_PHOTO_MISSION:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else {
            ROS_WARN("Ignoring command %s in state STARTING_PHOTO_MISSION", commandToStringMap.at(command));
        }
        break;
    case State::REACHING_MISSION_START_POINT:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else {
            ROS_WARN("Ignoring command %s in state REACHING_MISSION_START_POINT", commandToStringMap.at(command));
        }
        break;
    case State::TAKING_ORTHO_PHOTO:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else {
            ROS_WARN("Ignoring command %s in state TAKING_ORTHO_PHOTO", commandToStringMap.at(command));
        }
        break;
    case State::GENERATING_BOUNDARY:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else {
            ROS_WARN("Ignoring command %s in state GENERATING_BOUNDARY", commandToStringMap.at(command));
        }
        break;
    case State::COVERAGE_PATH_PLANNING:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else {
            ROS_WARN("Ignoring command %s in state COVERAGE_PATH_PLANNING", commandToStringMap.at(command));
        }
        break;
    case State::GENERATING_TRAJECTORY:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else {
            ROS_WARN("Ignoring command %s in state GENERATING_TRAJECTORY", commandToStringMap.at(command));
        }
        break;
    case State::COVERAGE_FLIGHT:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else {
            ROS_WARN("Ignoring command %s in state COVERAGE_FLIGHT", commandToStringMap.at(command));
        }
        break;
    case State::LANDING:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else {
            ROS_WARN("Ignoring command %s in state LANDING", commandToStringMap.at(command));
        }
        break;
    case State::MISSION_CANCELLING_RTL:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else {
            ROS_WARN("Ignoring command %s in state MISSION_CANCELLING_RTL", commandToStringMap.at(command));
        }
        break;

    }
}

void StateMachine::changeState(StateMachine::State newState) {
  ROS_INFO("Changing state from %s to %s", stateToStringMap.at(m_state), stateToStringMap.at(newState));
  m_state = newState;
}






/*
 * 
 * UTILTIES
 * 
 */

const std::map<StateMachine::State, const char *>  StateMachine::stateToStringMap = {
  { StateMachine::State::INIT, "INIT" },
  { StateMachine::State::READY, "READY" },
  { StateMachine::State::ARMING, "ARMING" },
  { StateMachine::State::TAKING_OFF, "TAKING_OFF" },
  { StateMachine::State::STARTING_PHOTO_MISSION, "STARTING_PHOTO_MISSION" },
  { StateMachine::State::REACHING_MISSION_START_POINT, "REACHING_MISSION_START_POINT" },
  { StateMachine::State::TAKING_ORTHO_PHOTO, "TAKING_ORTHO_PHOTO" },
  { StateMachine::State::GENERATING_BOUNDARY, "GENERATING_BOUNDARY" },
  { StateMachine::State::COVERAGE_PATH_PLANNING, "COVERAGE_PATH_PLANNING" },
  { StateMachine::State::GENERATING_TRAJECTORY, "GENERATING_TRAJECTORY" },
  { StateMachine::State::COVERAGE_FLIGHT, "COVERAGE_FLIGHT" },
  { StateMachine::State::LANDING, "LANDING" },
  { StateMachine::State::MISSION_CANCELLING_RTL, "MISSION_CANCELLING_RTL" },
};

const std::map<StateMachine::Command, const char *>  StateMachine::commandToStringMap = {
  { StateMachine::Command::MISSIONTRIGGER, "MISSIONTRIGGER" },
  { StateMachine::Command::GLOBAL_POSITION_UPDATE, "GLOBAL_POSITION_UPDATE" },
  { StateMachine::Command::TRY_ARM_TIMER_SHOT, "TRY_ARM_TIMER_SHOT" },
  { StateMachine::Command::UAV_MODE_UPDATE, "UAV_MODE_UPDATE" },
  { StateMachine::Command::MISSION_ITEM_REACHED, "MISSION_ITEM_REACHED" },
  { StateMachine::Command::ORTHO_PHOTO_READY, "ORTHO_PHOTO_READY" },
  { StateMachine::Command::BOUNDARY_GENERATED, "BOUNDARY_GENERATED" },
  { StateMachine::Command::COVERAGE_PATH_READY, "COVERAGE_PATH_READY" },
  { StateMachine::Command::TRAJECTORY_READY, "TRAJECTORY_READY" },
  { StateMachine::Command::BAMBI_FOUND, "BAMBI_FOUND" },
  { StateMachine::Command::BAMBI_SAVED, "BAMBI_SAVED" },
  { StateMachine::Command::COVERAGE_FC_REACHED_HOME, "COVERAGE_FC_REACHED_HOME" },
  { StateMachine::Command::LANDED_STATE_UPDATE, "LANDED_STATE_UPDATE" },
};

