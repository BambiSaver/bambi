/*
 * trajectorygeneratornode.h
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
#ifndef TRAJECTORYGENERATORNODE_H
#define TRAJECTORYGENERATORNODE_H

#include <ros/ros.h>
#include "bambi_msgs/PathWithConstraints.h"
#include <boost/geometry/geometries/point_xy.hpp>
#include <geographic_msgs/GeoPoint.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/PositionTarget.h>


typedef struct PointXYZ_relAltitude {
    double x;
    double y;
    uint32_t alt;
} PointXYZ_relAltitude;

typedef boost::geometry::model::d2::point_xy<double> point_xy_t;

namespace bambi {
namespace trajectory_generator{

class TrajectoryGeneratorNode
{
public:
    TrajectoryGeneratorNode(const ros::NodeHandle& nodeHandle);
    void spin();

private:
    void cb_trigger_trajectory_generation(const bambi_msgs::PathWithConstraints& pathWithConstraints);
    void cb_update_home_position(const mavros_msgs::HomePosition& homePosition);
    void generateTrajectory();
    void convertTrajectoryXYToGeoPoint(uint8_t zone, char band);
    void transformTrajectoryToLocalENU();
    double dist2D(const PointXYZ_relAltitude& p1,const PointXYZ_relAltitude& p2);
    ros::NodeHandle m_nodeHandle;
    ros::Publisher  m_publisherTrajectory;
    ros::Subscriber m_subscriberTriggerTrajectoryGeneration;
    ros::Subscriber m_subscriberHomePosition;
    float m_maxAcc;
    float m_maxVel;
    float m_setPointRate;

    mavros_msgs::HomePosition m_homePosition;
    bool m_homePositionReceived;

    std::vector<PointXYZ_relAltitude> m_pathXYZ_relAltitude;

    std::vector<mavros_msgs::PositionTarget> m_positionTrajectoryENU;
    std::vector<point_xy_t> m_path_xy;
    std::vector<point_xy_t> m_positionTrajectoryXY;

    std::vector<geographic_msgs::GeoPoint> m_positionTrajectoryGeo;
};
}
}
#endif // TRAJECTORYGENERATORNODE_H