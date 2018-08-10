/*
 * trajectorygeneratornode.cpp
 *
 * Created: 2018/8/9 by Michael Rimondi <michael.rimondi@outlook.it>
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
#ifndef COVERAGEPATHPLANNERNODE_H
#define COVERAGEPATHPLANNERNODE_H

#include <ros/ros.h>
#include <bambi_msgs/FieldCoverageInfo.h>

namespace bambi {
namespace coverage_path_planner{


class CoveragePathPlannerNode
{
public:
    CoveragePathPlannerNode(const ros::NodeHandle& nodeHandle);

    void spin();

private:
    void cb_trigger_path_generation(const bambi_msgs::FieldCoverageInfo& fieldCoverageInfo);
    ros::NodeHandle m_nodeHandle;
    ros::Publisher  m_publisherPath;
    ros::Subscriber m_subscriberTriggerPathGeneration;
};

}
}
#endif // COVERAGEPATHPLANNERNODE_H