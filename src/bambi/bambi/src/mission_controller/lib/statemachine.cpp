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
#include <bambi_msgs/FieldCoverageInfo.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/SetMode.h>
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
    if (command == Command::BOUNDARY_GENERATED) {
        // TODO MOVE THIS 'if' to State::GENERATING_BOUNDARY
        bambi_msgs::FieldCoverageInfo fieldWithInfo;
        fieldWithInfo.field = *(bambi_msgs::Field*)msg;
        // TODO get this info from m_missionTriggerStart
        fieldWithInfo.relative_altitude_scanning_in_mm = 8000;
        fieldWithInfo.relative_altitude_returning_in_mm = 15000;
        // 8x8m
        fieldWithInfo.thermal_camera_ground_footprint_height = 8.0f;
        fieldWithInfo.thermal_camera_ground_footprint_width = 8.0f;
<<<<<<< HEAD
        fieldWithInfo.home_position.latitude = 46.453066;
        fieldWithInfo.home_position.longitude = 11.492082;
=======
        fieldWithInfo.home_position.latitude = 46.452895;
        fieldWithInfo.home_position.longitude = 11.490920;
        fieldWithInfo.current_position.geopos_2d.latitude = 46.453066;
        fieldWithInfo.current_position.geopos_2d.longitude = 11.492082;
        fieldWithInfo.current_position.altitude_over_ground_in_mm = 70000;
>>>>>>> 91041263306f7a9a3eb4e595a64c6dfa1a1d3707
        changeState(State::COVERAGE_PATH_PLANNING);
        m_publisher.triggerPathGeneration(fieldWithInfo);
        return;
    }
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
                m_missionTriggerStart = *missionTriggerMsg;
                //float altitude = static_cast<float>(m_lastGlobalPosition.altitude);
                ROS_INFO("MISSIONTRIGGER received, sending ARM COMMAND");
                
                m_armingTries = 0;
                changeState(State::ARMING);
                // create and trigger arm timer
                m_armTimer = m_armTimerProviderFunction(ros::Duration(2.));
                m_armTimer.start();
                        
            } else {
                ROS_WARN("MISSIONTRIGGER STOP received: Mission not started yet, cannot STOP");
            }
        } else if (command == Command::GLOBAL_POSITION_UPDATE) {
            // silently ignore pos update, because not used here
        } else {
            ROS_WARN("Ignoring command %s in state READY", commandToStringMap.at(command));
        }
        break;
    case State::ARMING:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else if (command == Command::TRY_ARM_TIMER_SHOT) {
            if(m_armingTries < MAX_ARMING_TRIES){
                if (m_publisher.arm()){
                    ROS_INFO("Copter ARMED, sending takeoff message (altitude= %.2fm)", m_missionTriggerStart.altitude);
                    //save home position before sending takeoff request
                    m_homeGlobalPosition = m_lastGlobalPosition;
                    float globalAltitudeTO = static_cast<float>(m_lastGlobalPosition.altitude) + m_missionTriggerStart.altitude;
                    if (m_publisher.takeOff(globalAltitudeTO)) {
                        changeState(State::TAKING_OFF);
                    } else {
                        //Disarm it is not explicitly required as the PX4 disarm after a certein period of inactivity
                        ROS_WARN("TAKING OFF FAILED, going back to READY state");
                        changeState(State::READY);
                    }
                } else{
                    m_armTimer = m_armTimerProviderFunction(ros::Duration(2.));
                    m_armTimer.start();
                }
                m_armingTries++;
            } else{
                //no arm after MAX_ARMING_TRIES tries
                ROS_ERROR("NO ARMING after %d tries, going back to READY ", MAX_ARMING_TRIES);
                changeState(State::READY);
            }
        } else if (command == Command::GLOBAL_POSITION_UPDATE) {
            // silently ignore pos update, because not used here
        } else {
            ROS_WARN("Ignoring command %s in state ARMING", commandToStringMap.at(command));
        }
        break;

    case State::TAKING_OFF:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else if (command == Command::GLOBAL_POSITION_UPDATE) {
            // TODO tracking height to pass to next
            float lastRelativeAltitude = static_cast<float>(m_lastGlobalPosition.altitude - m_homeGlobalPosition.altitude);

            if (std::abs(lastRelativeAltitude-m_missionTriggerStart.altitude)< 0.1f){
                    //takeoff target altitude reached
                    if(m_publisher.clearWPList()){

                        mavros_msgs::Waypoint photoWP;
                        photoWP.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
                        photoWP.is_current = true;
                        photoWP.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
                        photoWP.x_lat = m_missionTriggerStart.latitude;
                        photoWP.y_long = m_missionTriggerStart.longitude;
                        photoWP.z_alt = static_cast<double>(m_missionTriggerStart.altitude); //relative altitude

                        mavros_msgs::WaypointPush listWP;
                        listWP.request.waypoints.push_back(photoWP);

                        if(m_publisher.pushWPList(listWP)){
                            //Waypoints list successfully sent
                            mavros_msgs::SetMode commandSetMode;
                            commandSetMode.request.base_mode = 1; //stands for MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                            commandSetMode.request.custom_mode = "AUTO.MISSION";
                            if(m_publisher.setMode(commandSetMode)){
                             //mission start command send (i.e change mode to AUTO.MISSION)
                                changeState(State::STARTING_PHOTO_MISSION);
                            }
                            else{
                                //Not able to change mode and start mission
                                changeState(State::READY);
                            }
                        }
                    }
            } else{
                //do nothing but wait for next globalPosition update
            }

        } else {
            ROS_WARN("Ignoring command %s in state TAKING_OFF", commandToStringMap.at(command));
        }
        break;

    case State::STARTING_PHOTO_MISSION:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");

        }else if (command == Command::UAV_MODE_UPDATE){
            if (m_lastUavMode == "AUTO.MISSION"){
                //mission has started
                changeState(State::REACHING_MISSION_START_POINT);
            }else{
                ROS_ERROR("Unexpected error in mode change going back to READY state");
                changeState(State::READY);
            }

        } else {
            ROS_WARN("Ignoring command %s in state STARTING_PHOTO_MISSION", commandToStringMap.at(command));
        }
        break;

    case State::REACHING_MISSION_START_POINT:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        }else if (command == Command::MISSION_ITEM_REACHED){
            //we have reached the mission starting point
            //TOD: SWITCH MODE TO LOITER (potresti avere problemi nel setMode, vedi https://github.com/mavlink/mavros/pull/811)


            changeState(State::TAKING_ORTHO_PHOTO);
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

