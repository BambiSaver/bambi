/*
 * publisher.h
 *
 * Created: 2018/08/07 by Florian Mahlknecht <m@florian.world>
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
#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <ros/ros.h>

#include <bambi_msgs/FieldCoverageInfo.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/SetMode.h>
namespace bambi {
namespace missioncontroller {

class MCPublisher
{
public:
    MCPublisher(const ros::NodeHandle& missioncontrollerNodeHandle);

    bool arm();
    bool takeOff(float takeoffAltitudeGlobal);
    bool clearWPList();
    bool pushWPList(mavros_msgs::WaypointPush &commandWPPush);
    bool setMode(mavros_msgs::SetMode &commandSetMode);
    void triggerPathGeneration(const bambi_msgs::FieldCoverageInfo& field);


private:
  ros::NodeHandle m_mcNodeHandle;
  ros::Publisher m_statusTextPublisher;
  ros::Publisher m_triggerShutterPublisher;
  ros::Publisher m_triggerBoundaryGenerationPublisher;
  ros::Publisher m_triggerPathGenerationPublisher;
  ros::Publisher m_triggerTrajectoryGenerationPublisher;
  ros::Publisher m_triggerCoverageFlightPublisher;
  ros::Publisher m_triggerHoverPublisher;
  ros::Publisher m_hoverPositionPublisher;
};

} // namespace missioncontroller
} // namespace bambi

#endif // PUBLISHER_H