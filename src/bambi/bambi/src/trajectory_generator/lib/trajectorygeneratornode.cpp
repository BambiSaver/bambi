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

#include <Splines.hh>
#include <cmath>


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

typedef boost::shared_ptr<SplinesLoad::CubicSpline> CubicSplinePtr;
typedef std::tuple<CubicSplinePtr, CubicSplinePtr, CubicSplinePtr> SplineCurve3d;

void pushBackSampleToSplineCurve(SplineCurve3d curve, double t, mavros_msgs::PositionTarget setPoint) {
    std::get<0>(curve)->pushBack(t, setPoint.position.x);
    std::get<1>(curve)->pushBack(t, setPoint.position.y);
    std::get<2>(curve)->pushBack(t, setPoint.position.z);
}

void TrajectoryGeneratorNode::generateTrajectory() {
    // TODO make SAVE for EMPTY m_pPathXYZ ?
    m_pPositionTrajectoryENU = boost::shared_ptr<std::vector<mavros_msgs::PositionTarget>>(new std::vector<mavros_msgs::PositionTarget>());

    std::tuple<CubicSplinePtr, CubicSplinePtr, CubicSplinePtr> curve {
        CubicSplinePtr(new SplinesLoad::CubicSpline()),
        CubicSplinePtr(new SplinesLoad::CubicSpline()),
        CubicSplinePtr(new SplinesLoad::CubicSpline())
    };

    double sampleTime = static_cast<double>(1) / m_setPointRate;

    double totalDist = 0.0;

    for (auto i = m_pPathXYZ_relAltitude->cbegin() + 1; i < m_pPathXYZ_relAltitude->cend(); ++i) {
        totalDist += dist2D(*(i-1), *i);
    }

    double distanceBetweenAPairOfSamples = static_cast<double>(m_maxVel) / m_setPointRate;

    auto numberOfElements = std::lround(totalDist / distanceBetweenAPairOfSamples);

    ROS_INFO("Path length = %.2f, to follow at %.2fm/s so we're trying to generate %ld samples", totalDist, m_maxVel, numberOfElements);

    // reserve AT LEAST numberOfElements
    m_pPositionTrajectoryENU->reserve(numberOfElements);

    //prepare PositionTarget message
    mavros_msgs::PositionTarget posTargetLocal;
    posTargetLocal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    posTargetLocal.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;


    double residualDistFromPreviousSegment = 0.0;
    auto i = m_pPathXYZ_relAltitude->cbegin();

    // insert first point
    posTargetLocal.position.x = i->x;
    posTargetLocal.position.y = i->y;
    posTargetLocal.position.z = i->alt;
    m_pPositionTrajectoryENU->push_back(posTargetLocal);
    pushBackSampleToSplineCurve(curve, 0.0, posTargetLocal);
    int samplesInserted = 1;

    bool reachedEnd = false;
    while (!reachedEnd) {
        auto j = i;

        double distanceBetweenChosenSegmentsWithResidual;
        do {
            ++j;
            if (j >= m_pPathXYZ_relAltitude->cend()) {
                // reached end;
                reachedEnd = true;
                break;
            }
            distanceBetweenChosenSegmentsWithResidual = residualDistFromPreviousSegment + dist2D(*i, *j);
        } while (distanceBetweenChosenSegmentsWithResidual < distanceBetweenAPairOfSamples);


        if (!reachedEnd) {
            // We've got to make N samples
            int N = static_cast<int>(std::floor(distanceBetweenChosenSegmentsWithResidual/distanceBetweenAPairOfSamples));
            Point3dRelAltitude unitDirectionVector;
            unitDirectionVector.x = (j->x - i->x) / dist2D(*i, *j);
            unitDirectionVector.y = (j->y - i->y) / dist2D(*i, *j);
            // N.B.: altitude is treated differently: we don't make residual considerations,
            // because the ground speed is limited, not the speed in 3D --> we're using dist2D()
            unitDirectionVector.alt = (j->alt - i->alt) / N;

            for (int k = 0; k < N; ++k) {
                posTargetLocal.position.x = i->x + ((k+1)*distanceBetweenAPairOfSamples - residualDistFromPreviousSegment) * unitDirectionVector.x;
                posTargetLocal.position.y = i->y + ((k+1)*distanceBetweenAPairOfSamples - residualDistFromPreviousSegment) * unitDirectionVector.y;
                posTargetLocal.position.z = i->alt + k * unitDirectionVector.alt;
                m_pPositionTrajectoryENU->push_back(posTargetLocal);
                pushBackSampleToSplineCurve(curve, sampleTime*samplesInserted, posTargetLocal);
                ++samplesInserted;
            }

            residualDistFromPreviousSegment = dist2D(
                    Point3dRelAltitude(m_pPositionTrajectoryENU->back().position.x, m_pPositionTrajectoryENU->back().position.y), *j);
            i = j;
        }
    }


    // insert last point
    posTargetLocal.position.x = m_pPathXYZ_relAltitude->back().x;
    posTargetLocal.position.y = m_pPathXYZ_relAltitude->back().y;
    posTargetLocal.position.z = m_pPathXYZ_relAltitude->back().alt;
    m_pPositionTrajectoryENU->push_back(posTargetLocal);
    pushBackSampleToSplineCurve(curve, sampleTime*samplesInserted, posTargetLocal);


    ROS_INFO("GENERATED %zd SAMPLES to be used at a frequency of %.1fHz", m_pPositionTrajectoryENU->size(), m_setPointRate);


    CubicSplinePtr curveX = std::get<0>(curve);
    CubicSplinePtr curveY = std::get<1>(curve);
    CubicSplinePtr curveZ = std::get<2>(curve);

    curveX->build();
    curveY->build();
    curveZ->build();


    for (int i = 0; i < m_pPositionTrajectoryENU->size(); ++i) {
        // better computation than addition in loop
        double t = sampleTime * i;
        mavros_msgs::PositionTarget& setpoint = m_pPositionTrajectoryENU->operator[](i);
        setpoint.velocity.x = curveX->D(t);
        setpoint.velocity.y = curveY->D(t);
        setpoint.velocity.z = curveZ->D(t);
        // head always towards where we are going
        setpoint.yaw = atan2(setpoint.velocity.y, setpoint.velocity.x);
        setpoint.yaw_rate = atan2(curveY->DD(t), curveX->DD(t));

//        if (i > 0) {
//            // calculate yaw for previous sample
//            mavros_msgs::PositionTarget& previousSetpoint = m_pPositionTrajectoryENU->operator[](i-1);
//            double deltaX = setpoint.velocity.x - previousSetpoint.velocity.x;
//            double deltaY = setpoint.velocity.y - previousSetpoint.velocity.y;
//            previousSetpoint.yaw_rate = atan2(deltaY, deltaX);
//        }
    }

//    m_pPositionTrajectoryENU->back().yaw_rate = 0.0;
}


double TrajectoryGeneratorNode::dist2D(const Point3dRelAltitude &p1, const Point3dRelAltitude &p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

