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
    m_subscriberHomePosition = m_nodeHandle.subscribe("/mavros/home_position/home",5,
                                                      &TrajectoryGeneratorNode::cb_update_home_position,this);
    m_subscriberTriggerTrajectoryGeneration = m_nodeHandle.subscribe("/bambi/mission_controller/trigger_trajectory_generation", 10,
                                                                     &TrajectoryGeneratorNode::cb_trigger_trajectory_generation, this);
    m_setPointRate = 40.0f;
}

void TrajectoryGeneratorNode::spin()
{
    ros::spin();
}




void TrajectoryGeneratorNode::cb_trigger_trajectory_generation(const bambi_msgs::PathWithConstraints &pathWithConstraints){
    m_maxAcc = pathWithConstraints.flight_constraints.max_acceleration;
    m_maxVel = pathWithConstraints.flight_constraints.max_velocity;



    //TODO check to have received at least one home position message

    //Transform home position in UTM to transoform path points in local referance system
    geographic_msgs::GeoPoint geoHomePoint;
    geoHomePoint.latitude = m_homePosition.geo.latitude;
    geoHomePoint.longitude = m_homePosition.geo.longitude;
    geodesy::UTMPoint utmHomePoint(geoHomePoint);

    for(auto Point : pathWithConstraints.path.geometric_path){

        geographic_msgs::GeoPoint geoPoint = geodesy::toMsg(Point.geopos_2d.latitude, Point.geopos_2d.longitude);
        geodesy::UTMPoint UTMPoint(geoPoint);
        PointXYZ_relAltitude pointXYZ_relAlt;
        pointXYZ_relAlt.x = UTMPoint.easting - utmHomePoint.easting;
        pointXYZ_relAlt.y = UTMPoint.northing - utmHomePoint.northing;
        pointXYZ_relAlt.alt = Point.altitude_over_ground;
        m_pathXYZ_relAltitude.push_back(pointXYZ_relAlt);
    }
    generateTrajectory();

    bambi_msgs::Trajectory bambiTrajectory;
    bambiTrajectory.sample_rate = m_setPointRate;
    bambiTrajectory.setpoints = m_positionTrajectoryENU;

    m_publisherTrajectory.publish(bambiTrajectory);
}

void TrajectoryGeneratorNode::cb_update_home_position(const mavros_msgs::HomePosition &homePosition)
{
    m_homePosition = homePosition;
    m_homePositionReceived = true;
}

void TrajectoryGeneratorNode::generateTrajectory()
{
    //preper PositionTarget message
    mavros_msgs::PositionTarget posTargetLocal;
    posTargetLocal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    posTargetLocal.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW ;

    bool lastElementReached = false;
    long N;
    size_t i = 0;
    size_t j = 0;
    size_t pathXYZSize = m_pathXYZ_relAltitude.size();

    while(i < pathXYZSize && !lastElementReached){
        j = i;
        N = 0;
        do{
            ++j;
            double dist = dist2D(m_pathXYZ_relAltitude[i], m_pathXYZ_relAltitude[j]);
            //N= number of generated setpoint in the current segment
            N = std::lround(m_setPointRate* dist/(m_maxVel) );

            if (j >= pathXYZSize - 1){
                lastElementReached = true;
            }
        //if we have no setpoint in the current segment we compute the distance to the next provided point
        }while(N <= 0 && !lastElementReached);

        if ( N==1 ){
            //if we have only one setpoint for the current segment we send the last poit of the segment
            posTargetLocal.position.x = m_pathXYZ_relAltitude[j].x;
            posTargetLocal.position.y = m_pathXYZ_relAltitude[j].y;
            posTargetLocal.position.z = m_pathXYZ_relAltitude[j].alt;
            m_positionTrajectoryENU.push_back(posTargetLocal);
        }else{
            for (size_t k = 0; k < N; ++k){
                //if we want N setpoint we must divide the segment in N-1 parts
                double t = k/(N-1);
                point_xy_t newSPoint;
                posTargetLocal.position.x = m_pathXYZ_relAltitude[i].x * (1-t) + m_pathXYZ_relAltitude[j].x * t;
                posTargetLocal.position.y = m_pathXYZ_relAltitude[i].y * (1-t) + m_pathXYZ_relAltitude[j].y * t;
                posTargetLocal.position.z = m_pathXYZ_relAltitude[i].alt * (1-t) + m_pathXYZ_relAltitude[j].alt *t;
                m_positionTrajectoryENU.push_back(posTargetLocal);
            }

        }

        ++i;

    }
    if(N <= 0 && lastElementReached){

        posTargetLocal.position.x = m_pathXYZ_relAltitude.back().x;
        posTargetLocal.position.y = m_pathXYZ_relAltitude.back().y;
        posTargetLocal.position.z = m_pathXYZ_relAltitude.back().alt;

        m_positionTrajectoryENU.push_back(posTargetLocal);
    }

}


double TrajectoryGeneratorNode::dist2D(const PointXYZ_relAltitude &p1, const PointXYZ_relAltitude &p2)
{
    return sqrt(pow(p1.x - p2.x, 2.) + pow(p1.y - p2.y, 2.));
}

