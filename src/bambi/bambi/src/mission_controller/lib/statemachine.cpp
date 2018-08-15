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
#include <bambi_msgs/CoverageFlightTrigger.h>
#include <mavros_msgs/StatusText.h>

//#define PATH_PLANNER_DEBUG_SETUP


using namespace bambi::missioncontroller;

StateMachine::StateMachine(const MCPublisher &publisher, rosTimerProviderFunction armTimerProvider,rosTimerProviderFunction changeModeTimerProvider) :
#ifdef PATH_PLANNER_DEBUG_SETUP
    m_state(State::GENERATING_BOUNDARY),
#else
    m_state(State::INIT),
#endif
    m_publisher(publisher),
    m_armTimerProviderFunction(armTimerProvider),
    m_changeModeTimerProviderFunction(changeModeTimerProvider),
    m_lastUavLandedState(mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED)
{


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

void StateMachine::cb_uav_altitude(const mavros_msgs::Altitude &msg) {
    ROS_INFO_THROTTLE(5, "Receiving altitude updates (relative to ground: %.2f)", msg.relative);
    m_lastAltitude = msg;
}
void StateMachine::cb_update_global_position(const sensor_msgs::NavSatFix &navSatFix) {
  // save in any case
  m_lastGlobalPosition = navSatFix;
  handleStateMachineCommand(Command::GLOBAL_POSITION_UPDATE, &navSatFix);
}
void StateMachine::cb_arming_timer(const ros::TimerEvent &) {
    handleStateMachineCommand(Command::TRY_ARM_TIMER_SHOT, NULL);
}

void StateMachine::cb_change_mode_timer(const ros::TimerEvent &)
{
     handleStateMachineCommand(Command::CHANGE_MODE_TIMER_SHOT, NULL);
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
                bambiInfo("Gps get lock => Global Position received");
                changeState(State::READY);
            } else {
                ROS_DEBUG("GPS fix received, but status is not STATUS_FIX, so waiting for next to change to READY");
            }
        } else {
            ROS_WARN("Ignoring command %s in state INIT", commandToStringMap.at(command));
        }
        break;
    case State::READY:
        if (command == Command::MISSIONTRIGGER) {
            auto missionTriggerMsg = (mavros_msgs::BambiMissionTrigger*)msg;
            bambiInfo("MTrig %1d=%4.1f=%6.2f=%6.2f=%4.1f=%4.1f=%6.1f",
                      static_cast<int>(missionTriggerMsg->startStop),
                      missionTriggerMsg->altitudeTakeoff,
                      missionTriggerMsg->latitudeOrthophoto,
                      missionTriggerMsg->longitudeOrthophoto,
                      missionTriggerMsg->SensorFootprintMinDim,
                      missionTriggerMsg->altitudeScanning,
                      missionTriggerMsg->altitudeOrthophoto);
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
                bambiInfo("Arming try %d/%d", m_armingTries, MAX_ARMING_TRIES);
                if (m_publisher.arm()){
                    ROS_INFO("Copter ARMED, sending takeoff message (altitude= %.2fm)", m_missionTriggerStart.altitudeTakeoff);
                    //save home position before sending takeoff request
                    m_homeGlobalPosition = m_lastGlobalPosition;
                    float globalAltitudeTO = m_lastAltitude.amsl + m_missionTriggerStart.altitudeTakeoff;
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
                bambiError("ARM FAILED after %d tries", MAX_ARMING_TRIES);
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

            if (m_lastAltitude.relative > m_missionTriggerStart.altitudeTakeoff-0.1f){
                    bambiInfo("TO altitude %4.1fm reached", m_lastAltitude.relative);
                    //takeoff target altitude reached
                    if(m_publisher.clearWPList()){
                        mavros_msgs::Waypoint photoWP;
                        photoWP.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
                        photoWP.is_current = true;
                        photoWP.frame = mavros_msgs::Waypoint::FRAME_GLOBAL;
                        photoWP.x_lat = m_missionTriggerStart.latitudeOrthophoto;
                        photoWP.y_long = m_missionTriggerStart.longitudeOrthophoto;
                        //Altitude should be expressed in AMSL (meters)
                        photoWP.z_alt = m_missionTriggerStart.altitudeOrthophoto;
                        //create WPlist
                        mavros_msgs::WaypointPush listWP;
                        listWP.request.waypoints.push_back(photoWP);
                        if(m_publisher.pushWPList(listWP)){
                            //Waypoints list successfully sent
                            bambiInfo("WP sent lat=%6.2f long=%6.2f alt=%6.1f",listWP.request.waypoints[0].x_lat,
                                    listWP.request.waypoints[0].y_long,
                                    listWP.request.waypoints[0].z_alt);
                            //wait 2 second before change mode to auto
                            m_changeModeTimer = m_changeModeTimerProviderFunction(ros::Duration(2.));
                            m_changeModeTimer.start();
                            changeState(State::CHANGING_TO_AUTO_MODE);
                        } else{
                            bambiError("Not able to send WP list => READY");
                            //Not able to change mode and start mission
                            changeState(State::READY);
                            }
                    } else{
                        bambiError("Not able to wipe WPs list => READY");
                        //Not able to wipe FCU WPs list
                        changeState(State::READY);
                }
            } else{
                //do nothing but wait for next globalPosition update
            }

        } else {
            ROS_WARN("Ignoring command %s in state TAKING_OFF", commandToStringMap.at(command));
        }
        break;

    case State::CHANGING_TO_AUTO_MODE:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else if (command == Command::CHANGE_MODE_TIMER_SHOT) {
            //change mode to AUTO.MISSION
            bambiInfo("Request to change mode to AUTO.MISSION");
            mavros_msgs::SetMode commandSetMode;
            commandSetMode.request.base_mode = 1; //stands for MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            commandSetMode.request.custom_mode = "AUTO.MISSION";
            if(m_publisher.setMode(commandSetMode)){
                //mission start_command send (i.e change mode to AUTO.MISSION)
                changeState(State::STARTING_PHOTO_MISSION);
            } else{
                bambiError("UNABLE TO CHANGE MODE => READY");
                //Not able to change mode and start mission
                changeState(State::READY);
            }
        } else if (command == Command::GLOBAL_POSITION_UPDATE // savely ignore GPS update, because we are not tracking any position here
              || command == Command::MISSION_ITEM_REACHED // remaining MISSION_ITEM_REACHED may arrive, all OK
              ) {
            // ignore
        } else {
            ROS_WARN("Ignoring command %s in state CHANGING_TO_AUTO_MODE", commandToStringMap.at(command));
        }
        break;    
    case State::STARTING_PHOTO_MISSION:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");

        }else if (command == Command::UAV_MODE_UPDATE){
            if (m_lastUavMode == "AUTO.MISSION"){
                bambiInfo("UAV mode= %s", m_lastUavMode.c_str());
                //mission has started
                changeState(State::REACHING_MISSION_START_POINT);
            }else{
                // TODO RTL here?
                bambiError("UAV MODE NOT CHANGED => READY");
                ROS_ERROR("Unexpected error in mode change going back to READY state");

                changeState(State::READY);
            }

        } else if (command == Command::GLOBAL_POSITION_UPDATE) {
            // savely ignore GPS update, because we are waiting for a vehicle state change here
        } else {
            ROS_WARN("Ignoring command %s in state STARTING_PHOTO_MISSION", commandToStringMap.at(command));
        }
        break;

    case State::REACHING_MISSION_START_POINT:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else if (command == Command::MISSION_ITEM_REACHED){
            bambiInfo("WP reached");
            //we have reached the mission starting point
            //???TODO???: SWITCH MODE TO LOITER (potresti avere problemi nel setMode, vedi https://github.com/mavlink/mavros/pull/811)

            changeState(State::TAKING_ORTHO_PHOTO);
            m_publisher.triggerOrthPhotoShutter();
        } else if (command == Command::GLOBAL_POSITION_UPDATE) {
            // silently ignore GPS update, because we are waiting for MISSION_ITEM_REACHED
        } else {
            ROS_WARN("Ignoring command %s in state REACHING_MISSION_START_POINT", commandToStringMap.at(command));
        }
        break;

    case State::TAKING_ORTHO_PHOTO:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else if (command == Command::ORTHO_PHOTO_READY) {
            auto photo = (bambi_msgs::OrthoPhoto*)msg;
            bambiInfo("Orthophoto Saved");
            changeState(State::GENERATING_BOUNDARY);
            m_publisher.triggerBoundaryGeneration(*photo);

        } else if (command == Command::GLOBAL_POSITION_UPDATE // savely ignore GPS update, because we are not tracking any position here
              || command == Command::MISSION_ITEM_REACHED // remaining MISSION_ITEM_REACHED may arrive, all OK
              ) {
            // ignore
        } else {
            ROS_WARN("Ignoring command %s in state TAKING_ORTHO_PHOTO", commandToStringMap.at(command));
        }
        break;
    case State::GENERATING_BOUNDARY:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else if (command == Command::BOUNDARY_GENERATED) {

            bambiInfo("CPP AP=%6.2f %6.2f %4.1f HP=%6.2f %6.2f %4.1f", m_homeGlobalPosition.latitude,
                      m_homeGlobalPosition.longitude,
                      m_lastAltitude.relative,
                      m_homeGlobalPosition.latitude,
                      m_homeGlobalPosition.longitude);

            bambi_msgs::FieldCoverageInfo fieldWithInfo;

            fieldWithInfo.field = *(bambi_msgs::Field*)msg;
            fieldWithInfo.relative_altitude_scanning = m_missionTriggerStart.altitudeScanning;
            fieldWithInfo.relative_altitude_returning = m_missionTriggerStart.altitudeTakeoff;
            //TODO check if altitude msg m_lastAltitude.terrain is correct then use that as altitude over ground.
            //if, in case of no distance sensor the information it is avaiable we could use relative altitude.
            fieldWithInfo.current_position.altitude_over_ground = m_lastAltitude.relative;
            fieldWithInfo.thermal_camera_ground_footprint_height = m_missionTriggerStart.SensorFootprintMinDim;
            fieldWithInfo.thermal_camera_ground_footprint_width = m_missionTriggerStart.SensorFootprintMinDim;
            fieldWithInfo.home_position.latitude = m_homeGlobalPosition.latitude;
            fieldWithInfo.home_position.longitude = m_homeGlobalPosition.longitude;
            fieldWithInfo.current_position.geopos_2d.latitude = m_lastGlobalPosition.latitude;
            fieldWithInfo.current_position.geopos_2d.longitude = m_lastGlobalPosition.longitude;

#ifdef PATH_PLANNER_DEBUG_SETUP
            fieldWithInfo.current_position.altitude_over_ground = 45000;
            fieldWithInfo.current_position.geopos_2d.latitude = 46.453072;
            fieldWithInfo.current_position.geopos_2d.longitude = 11.492048;
            fieldWithInfo.home_position.latitude = 46.452895;
            fieldWithInfo.home_position.longitude = 11.490920;
            fieldWithInfo.thermal_camera_ground_footprint_height = 8.0;
            fieldWithInfo.thermal_camera_ground_footprint_width = 8.0;
#endif

            changeState(State::COVERAGE_PATH_PLANNING);
            m_publisher.triggerPathGeneration(fieldWithInfo);
        }  else if (command == Command::GLOBAL_POSITION_UPDATE // savely ignore GPS update, because we are not tracking any position here
                    || command == Command::MISSION_ITEM_REACHED // remaining MISSION_ITEM_REACHED may arrive, all OK
                    ) {
            // ignore
        }else {
            ROS_WARN("Ignoring command %s in state GENERATING_BOUNDARY", commandToStringMap.at(command));
        }
        break;
    case State::COVERAGE_PATH_PLANNING:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else if (command == Command::COVERAGE_PATH_READY) {
            changeState(State::GENERATING_TRAJECTORY);
            ROS_INFO("INVOKING TRAJECTORY GENERATOR");
            bambi_msgs::PathWithConstraints pathWithConstraints;
            pathWithConstraints.path = *((bambi_msgs::Path*)msg);

            // TODO get from m_missionTriggerStart (?)
            pathWithConstraints.flight_constraints.max_velocity = 5.0;
            pathWithConstraints.flight_constraints.max_acceleration = 15.0;
            m_publisher.triggerTrajectoryGeneration(pathWithConstraints);

            bambiInfo("Path pts=%zd Vmax=%3.1f Amax=%4.1f",pathWithConstraints.path.geometric_path.size(),
                      pathWithConstraints.flight_constraints.max_velocity,
                      pathWithConstraints.flight_constraints.max_acceleration);
        } else if (command == Command::GLOBAL_POSITION_UPDATE // savely ignore GPS update, because we are not tracking any position here
                   || command == Command::MISSION_ITEM_REACHED // remaining MISSION_ITEM_REACHED may arrive, all OK
              ) {
            // ignore
        } else {
            ROS_WARN("Ignoring command %s in state COVERAGE_PATH_PLANNING", commandToStringMap.at(command));
        }
        break;
    case State::GENERATING_TRAJECTORY:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else if (command == Command::TRAJECTORY_READY) {
            changeState(State::COVERAGE_FLIGHT);
            auto trajectory = (bambi_msgs::Trajectory*) msg;
            bambi_msgs::CoverageFlightTrigger cft;
            cft.startStop =true;
            cft.trajectory = *trajectory;
            m_publisher.triggerCoverageFlight(cft);
            bambiInfo("CFT Pts=%zd SamplRate=%5.1f", cft.trajectory.setpoints.size(), cft.trajectory.sample_rate);
        } else if (command == Command::GLOBAL_POSITION_UPDATE // savely ignore GPS update, because we are not tracking any position here
                   || command == Command::MISSION_ITEM_REACHED // remaining MISSION_ITEM_REACHED may arrive, all OK
              ) {
            // ignore
        } else {
            ROS_WARN("Ignoring command %s in state GENERATING_TRAJECTORY", commandToStringMap.at(command));
        }
        break;
    case State::COVERAGE_FLIGHT:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else if (command == Command::COVERAGE_FC_REACHED_HOME) {
            bambiInfo("Home reached!!! Almost done fellow");
            ROS_INFO("We reached HOME!!!!");
            mavros_msgs::SetMode commandSetMode;
            commandSetMode.request.base_mode = 1; //stands for MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            commandSetMode.request.custom_mode = "AUTO.LAND";
            if(m_publisher.setMode(commandSetMode)){
                // say to flight_controller to stop publishing setpoints
                bambi_msgs::CoverageFlightTrigger ft;
                ft.startStop = false;
                m_publisher.triggerCoverageFlight(ft);
                //mission start command send (i.e change mode to AUTO.MISSION)
                changeState(State::LANDING);
            } else {
                bambiError("CANNOT SET AUTO.LAND mode => READY state");
                // TODO RTL?
                //Not able to change mode and start mission
                changeState(State::READY);
                ROS_WARN("LANDING REJECTED");
            }
        } else if (command == Command::GLOBAL_POSITION_UPDATE // not used for now, maybe in FlightController
                    ) {
            // ignore
        } else {
            ROS_WARN("Ignoring command %s in state COVERAGE_FLIGHT", commandToStringMap.at(command));
        }
        break;
    case State::LANDING:
        if (command == Command::MISSIONTRIGGER) {
            ROS_INFO("Mission trigger received but not handled");
        } else if (command == Command::LANDED_STATE_UPDATE) {
            // auto extendedState = static_cast<mavros_msgs::ExtendedState*>(msg); NOT NECESSARY BECAUSE WE HAVE IT IN MEMBER
            if (m_lastUavLandedState == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
                bambiInfo("Successfully landed(!), SWITCH ME OFF PLZ");
                ROS_INFO("Successfully landed(!)");
                changeState(State::READY);
            }
        } else if (command == Command::GLOBAL_POSITION_UPDATE // not used for now, maybe in FlightController
                   ) {
           // ignore
        } else {
            ROS_WARN("Ignoring command %s in state LANDING", commandToStringMap.at(command));
        }
        break;
    case State::MISSION_CANCELLING_RTL:
        if (command == Command::MISSIONTRIGGER) {
            bambiWarn("Mission Aborted RTL now!!!");


            ROS_INFO("Mission trigger received but not handled");
        } else {
            ROS_WARN("Ignoring command %s in state MISSION_CANCELLING_RTL", commandToStringMap.at(command));
        }
        break;
    }
}

void StateMachine::changeState(StateMachine::State newState) {
  ROS_INFO("STATE MACHINE STATE CHANGE %s ==> %s", stateToStringMap.at(m_state), stateToStringMap.at(newState));
  m_state = newState;
}


void StateMachine::bambiInfo(const char format[], ...){

    std::string msg;
    msg = "BB_" + stateToLogStringMap.at(m_state) + "-";
    va_list args;
    char buffer[128];
    sprintf(buffer, format, args);
    va_end(args);
    msg = msg + buffer;
    m_publisher.sendStatusText(msg,mavros_msgs::StatusText::INFO);

}

void StateMachine::bambiDebug(const char format[], ...){
    std::string msg;
    msg = "BB_" + stateToLogStringMap.at(m_state) + "-";
    va_list args;
    char buffer[128];
    sprintf(buffer, format, args);
    va_end(args);
    msg = msg + buffer;
    m_publisher.sendStatusText(msg,mavros_msgs::StatusText::DEBUG);

}

void StateMachine::bambiWarn(const char format[], ...){
    std::string msg;
    msg = "BB_" + stateToLogStringMap.at(m_state) + "-";
    va_list args;
    char buffer[128];
    sprintf(buffer, format, args);
    va_end(args);
    msg = msg + buffer;
    m_publisher.sendStatusText(msg,mavros_msgs::StatusText::ALERT);
}

void StateMachine::bambiError(const char format[], ...){
    std::string msg;
    msg = "BB_" + stateToLogStringMap.at(m_state) + "-";
    va_list args;
    char buffer[128];
    sprintf(buffer, format, args);
    va_end(args);
    msg = msg + buffer;
    m_publisher.sendStatusText(msg,mavros_msgs::StatusText::ERROR);
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
  { StateMachine::State::CHANGING_TO_AUTO_MODE, "CHANGING_TO_AUTO_MODE"},
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
  { StateMachine::Command::CHANGE_MODE_TIMER_SHOT, "CHANGE_MODE_TIMER_SHOT" },
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



const std::map<StateMachine::State, std::string>  StateMachine::stateToLogStringMap = {
  { StateMachine::State::INIT, "INIT" },
  { StateMachine::State::READY, "READY" },
  { StateMachine::State::ARMING, "ARMING" },
  { StateMachine::State::TAKING_OFF, "TOING" },
  { StateMachine::State::CHANGING_TO_AUTO_MODE, "CTAM"},
  { StateMachine::State::STARTING_PHOTO_MISSION, "SPM" },
  { StateMachine::State::REACHING_MISSION_START_POINT, "RMSP" },
  { StateMachine::State::TAKING_ORTHO_PHOTO, "TOP" },
  { StateMachine::State::GENERATING_BOUNDARY, "GENB" },
  { StateMachine::State::COVERAGE_PATH_PLANNING, "CPP" },
  { StateMachine::State::GENERATING_TRAJECTORY, "GENTR" },
  { StateMachine::State::COVERAGE_FLIGHT, "COVFL" },
  { StateMachine::State::LANDING, "LNDNG" },
  { StateMachine::State::MISSION_CANCELLING_RTL, "MCRTL" },
};