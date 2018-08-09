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
#include "coveragepathplannernode.h"
#include <bambi_msgs/Path.h>


using namespace bambi::coverage_path_planner;


CoveragePathPlannerNode::CoveragePathPlannerNode(const ros::NodeHandle &nodeHandle)
    : m_nodeHandle(nodeHandle) {

    m_publisherPath = m_nodeHandle.advertise<bambi_msgs::Path>("path", 5, false);

    m_subscriberTriggerPathGeneration = m_nodeHandle.subscribe("/bambi/mission_controller/trigger_path_generation", 10,
                                                               &CoveragePathPlannerNode::cb_trigger_path_generation, this);
}

void CoveragePathPlannerNode::spin()
{
    ros::spin();
}

void CoveragePathPlannerNode::cb_trigger_path_generation(const bambi_msgs::FieldCoverageInfo &fieldCoverageInfo) {
    ROS_INFO("Coverage Path Planning node got field coverage info");
}
