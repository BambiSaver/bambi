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
    m_setPointRate = 50.0f;
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

    m_pPathXYZ_relAltitude = boost::shared_ptr<std::vector<Point3dRelAltitude>>(new std::vector<Point3dRelAltitude>());
    m_pPathXYZ_relAltitude->reserve(pathWithConstraints.path.geometric_path.size());

    for(auto point : pathWithConstraints.path.geometric_path){
        geographic_msgs::GeoPoint geoPoint = geodesy::toMsg(point.geopos_2d.latitude, point.geopos_2d.longitude);
        geodesy::UTMPoint UTMPoint(geoPoint);
        Point3dRelAltitude pointXYZ_relAlt;
        pointXYZ_relAlt.x = UTMPoint.easting - utmHomePoint.easting;
        pointXYZ_relAlt.y = UTMPoint.northing - utmHomePoint.northing;
        pointXYZ_relAlt.alt = point.altitude_over_ground;
        m_pPathXYZ_relAltitude->push_back(pointXYZ_relAlt);
    }

    generateTrajectory();

    auto bambiTrajectory = boost::shared_ptr<bambi_msgs::Trajectory>(new bambi_msgs::Trajectory());
    bambiTrajectory->sample_rate = m_setPointRate;
    bambiTrajectory->setpoints = *m_pPositionTrajectoryENU;

    m_publisherTrajectory.publish(bambiTrajectory);
}

void TrajectoryGeneratorNode::cb_update_home_position(const mavros_msgs::HomePosition &homePosition)
{
    m_homePosition = homePosition;
}

void TrajectoryGeneratorNode::generateTrajectory()
{
    m_pPositionTrajectoryENU = boost::shared_ptr<std::vector<mavros_msgs::PositionTarget>>(new std::vector<mavros_msgs::PositionTarget>());

    // reserve AT LEAST the same size
    m_pPositionTrajectoryENU->reserve(m_pPathXYZ_relAltitude->size());

    //prepare PositionTarget message
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
    size_t pathXYZSize = m_pPathXYZ_relAltitude->size();

    double totalDist = 0.0;

    while(i < pathXYZSize && !lastElementReached){
        j = i;
        N = 0;
        do{
            ++j;

            double dist = dist2D(m_pPathXYZ_relAltitude->at(i), m_pPathXYZ_relAltitude->at(j));

            //N= number of generated setpoint in the current segment
            N = static_cast<int>(std::floor(m_setPointRate* dist/(m_maxVel)));


            if (j >= pathXYZSize - 1){
                lastElementReached = true;
            }
        //if we have no setpoint in the current segment we compute the distance to the next provided point
        }while(N <= 0 && !lastElementReached);

        totalDist += dist2D(m_pPathXYZ_relAltitude->at(i), m_pPathXYZ_relAltitude->at(j));

        if ( N==1 ){
            if (j >= pathXYZSize) {
                ROS_WARN("CHE BOTTA VEZ");
            }
            //if we have only one setpoint for the current segment we send the last poit of the segment
            posTargetLocal.position.x = m_pPathXYZ_relAltitude->at(j).x;
            posTargetLocal.position.y = m_pPathXYZ_relAltitude->at(j).y;
            posTargetLocal.position.z = m_pPathXYZ_relAltitude->at(j).alt;
            m_pPositionTrajectoryENU->push_back(posTargetLocal);
        }else{
            for (size_t k = 0; k < N; ++k){
                //if we want N setpoint we must divide the segment in N-1 parts
                double t = k/(N-1);

                if (i >= pathXYZSize || j >= pathXYZSize) {
                    ROS_WARN("CHE BOTTA VEZ");
                }

                posTargetLocal.position.x = m_pPathXYZ_relAltitude->at(i).x * (1-t) + m_pPathXYZ_relAltitude->at(j).x * t;
                posTargetLocal.position.y = m_pPathXYZ_relAltitude->at(i).y * (1-t) + m_pPathXYZ_relAltitude->at(j).y * t;
                posTargetLocal.position.z = m_pPathXYZ_relAltitude->at(i).alt * (1-t) + m_pPathXYZ_relAltitude->at(j).alt *t;
                m_pPositionTrajectoryENU->push_back(posTargetLocal);
            }

        }

        i=j;

    }
    if(N <= 0 && lastElementReached){

        posTargetLocal.position.x = m_pPathXYZ_relAltitude->back().x;
        posTargetLocal.position.y = m_pPathXYZ_relAltitude->back().y;
        posTargetLocal.position.z = m_pPathXYZ_relAltitude->back().alt;

        m_pPositionTrajectoryENU->push_back(posTargetLocal);
    }


    ROS_INFO("%.2f / %.2f * %.2f = %.2f", totalDist, m_maxVel, m_setPointRate, std::floor(totalDist / m_maxVel * m_setPointRate));

    ROS_INFO("Path length = %.2f", totalDist);

    ROS_INFO("GOT MANY POINTS: %d", static_cast<int>(m_pPositionTrajectoryENU->size()));
}


double TrajectoryGeneratorNode::dist2D(const Point3dRelAltitude &p1, const Point3dRelAltitude &p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

