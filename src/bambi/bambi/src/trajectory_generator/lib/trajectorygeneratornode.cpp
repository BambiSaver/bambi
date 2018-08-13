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
#include "trajectorygeneratornode.h"
#include "bambi_msgs/Trajectory.h"

#include <geodesy/utm.h>
#include <boost/geometry/algorithms/distance.hpp>

#include <geographic_msgs/GeoPoint.h>



using namespace bambi::trajectory_generator;

TrajectoryGeneratorNode::TrajectoryGeneratorNode(const ros::NodeHandle& nodeHandle)
    :m_nodeHandle(nodeHandle)
{

    m_publisherTrajectory = m_nodeHandle.advertise<bambi_msgs::Trajectory>("trajectory", 5, false);

    m_subscriberTriggerTrajectoryGeneration = m_nodeHandle.subscribe("/bambi/mission_controller/trigger_trajectory_generation", 10, &TrajectoryGeneratorNode::cb_trigger_trajectory_generation, this);
}

void TrajectoryGeneratorNode::spin()
{
    ros::spin();
}

void TrajectoryGeneratorNode::cb_trigger_trajectory_generation(const bambi_msgs::PathWithConstraints &pathWithConstraints)
{
    for(auto Point : pathWithConstraints.path.geometric_path){

        geographic_msgs::GeoPoint geoPoint = geodesy::toMsg(Point.geopos_2d.latitude, Point.geopos_2d.longitude);
        geodesy::UTMPoint UTMPoint(geoPoint);
        point_xy_t point_xy(UTMPoint.easting, UTMPoint.northing);
        m_path_xy.push_back(point_xy);
    }
    m_maxAcc = pathWithConstraints.flight_constraints.max_acceleration;
    m_maxVel = pathWithConstraints.flight_constraints.max_velocity;

    ROS_INFO("Trajectory generator got path with constraints messages");
}

void TrajectoryGeneratorNode::generateTrajectory()
{
    const float safeVelFactor = 0.8f;
    bool lastElementReached = false;
    std::vector<point_xy_t> trajectoryXY;
    long N;
    size_t i = 0;
    size_t j = 0;
    size_t pathXYSize = m_path_xy.size();
    while(i < pathXYSize && !lastElementReached){
        j = i;
        N = 0;
        do{
            ++j;
            double dist = boost::geometry::distance(m_path_xy[i],m_path_xy[j]);
            //N= number of generated setpoint in the current segment
            N = std::lround(m_setPointRate* dist/(m_maxVel * safeVelFactor) );

            if (j >= pathXYSize){
                lastElementReached = true;
            }
        //if we have no setpoint in the current segment we compute the distance to the next provided point
        }while(N <= 0 && !lastElementReached);


        for (size_t k = 0; k < N; ++k){
            //if we want N setpoint we must divide the segment in N-1 parts
            double t = k/(N-1);
            point_xy_t newSPoint;
            newSPoint.set<0>(m_path_xy[i].get<0>() * (1-t) + m_path_xy[j].get<0>() * t);
            newSPoint.set<1>(m_path_xy[i].get<1>() * (1-t) + m_path_xy[j].get<1>() * t);
            trajectoryXY.push_back(newSPoint);
        }

        ++i;

    }
    if(N <= 0 && lastElementReached){
        trajectoryXY.push_back(m_path_xy.back());
    }


    m_positionTrajectory = trajectoryXY;
}
