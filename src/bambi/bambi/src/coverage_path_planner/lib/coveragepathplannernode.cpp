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

#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>
#include <algorithm>
#include <map>

#include <deque>

#include <boost/shared_ptr.hpp>
#include <boost/multi_array.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using namespace bambi::coverage_path_planner;

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
//typedef boost::geometry::model::d2::point_xy<double> point_t;
typedef boost::geometry::model::polygon<point_t> polygon_t;
typedef std::map<boost::tuple<int, int>, polygon_t> map_t;


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

polygon_t helperFunctionGetSquarePolygonFromMatrixCell(double start_N, double start_E, int i, int j, float sensorFootprint) {
    polygon_t p;

    std::vector<point_t> points;
    points.push_back(point_t(start_N + i*sensorFootprint, start_E + j*sensorFootprint));
    points.push_back(point_t(start_N + i*sensorFootprint, start_E + (j+1)*sensorFootprint));
    points.push_back(point_t(start_N + (i+1)*sensorFootprint, start_E + (j+1)*sensorFootprint));
    points.push_back(point_t(start_N + (i+1)*sensorFootprint, start_E + j*sensorFootprint));
    points.push_back(point_t(start_N + i*sensorFootprint, start_E + j*sensorFootprint));

    //boost::geometry::set()
    boost::geometry::assign_points(p, points);
    return p;
}

void CoveragePathPlannerNode::cb_trigger_path_generation(const bambi_msgs::FieldCoverageInfo &fieldCoverageInfo) {

    // TODO: MAKE SAFE FOR EMPTY BOUNDARY ARRAY (coredumps for now)


    ROS_INFO("Got field coverage info, with a path of %d coordinates, flight heights (%.1f, %.1f) and sensor footprint %.1f x %.1f"
             , static_cast<int>(fieldCoverageInfo.field.boundary_path.size()), fieldCoverageInfo.relative_altitude_returning_in_mm / 1E3
             , fieldCoverageInfo.relative_altitude_scanning_in_mm / 1E3, fieldCoverageInfo.thermal_camera_ground_footprint_width
             , fieldCoverageInfo.thermal_camera_ground_footprint_height);

    //std::vector<geodesy::UTMPoint> utmPoints;
    std::vector<double> latitudes;
    std::vector<double> longitudes;
    polygon_t boostFieldBorderPolygon;

    for (auto point : fieldCoverageInfo.field.boundary_path) {
        geographic_msgs::GeoPoint geoPoint = geodesy::toMsg(point.latitude, point.longitude);
        latitudes.push_back(point.latitude);
        longitudes.push_back(point.longitude);
        geodesy::UTMPoint utmPoint((geoPoint));
        //utmPoints.push_back(geodesy::UTMPoint);
        boost::geometry::append(boostFieldBorderPolygon.outer(), point_t(utmPoint.northing, utmPoint.easting));
        //ROS_INFO("(LAT, LON) = (%f, %f)", point.latitude, point.longitude);
    }

    double latitude_min = *std::min_element(latitudes.begin(),latitudes.end());
    double latitude_max = *std::max_element(latitudes.begin(),latitudes.end());

    double longitude_min = *std::min_element(longitudes.begin(),longitudes.end());
    double longitude_max = *std::max_element(longitudes.begin(),longitudes.end());

//    ROS_INFO("LAT in [%.8f, %.8f]", latitude_min, latitude_max);
//    ROS_INFO("LON in [%.8f, %.8f]", longitude_min, longitude_max);

    geodesy::UTMPoint bottomLeft(geodesy::toMsg(latitude_min, longitude_min));
    geodesy::UTMPoint bottomRight(geodesy::toMsg(latitude_min, longitude_max));
    geodesy::UTMPoint topLeft(geodesy::toMsg(latitude_max, longitude_min));
    geodesy::UTMPoint topRight(geodesy::toMsg(latitude_max, longitude_max));

//    ROS_INFO("bottomLeft: (%f, %f)", bottomLeft.easting, bottomLeft.northing);
//    ROS_INFO("bottomRight: (%f, %f)", bottomRight.easting, bottomRight.northing);
//    ROS_INFO("upLeft: (%f, %f)", topLeft.easting, topLeft.northing);
//    ROS_INFO("upRight: (%f, %f)", topRight.easting, topRight.northing);

    // due to distortion they are NOT equal, even though we are coming from the same longitudine
    double leftBorder_E = std::min(bottomLeft.easting, topRight.easting);
    double bottomBorder_N = std::min(bottomLeft.northing, bottomRight.northing);
    double rightBorder_E = std::max(bottomRight.easting, topRight.easting);
    double topBorder_N = std::max(topLeft.northing, topRight.northing);

    double width = rightBorder_E - leftBorder_E;
    double height = topBorder_N - bottomBorder_N;

    ROS_INFO("FIELD DIMENSION: %.2fm x %.2fm", width, height);

    float minDimFootprint = std::min(fieldCoverageInfo.thermal_camera_ground_footprint_width, fieldCoverageInfo.thermal_camera_ground_footprint_height);
    int n_E = std::ceil(width/minDimFootprint);
    int n_N = std::ceil(height/minDimFootprint);

    ROS_INFO("Making a grid of %d x %d cells, because we have a sensor footprint of %.2fm x %.2fm", n_E, n_N, minDimFootprint, minDimFootprint);

    boost::multi_array<int, 2> matrix(boost::extents[n_N][n_E]);
    //boost::shared_ptr<map_t> map(new map_t());


    double fieldArea = boost::geometry::area(boostFieldBorderPolygon);
    ROS_INFO("FIELD IS %.2fm^2", fieldArea);


    for (int i = 0; i < n_N; ++i) {
        for (int j = 0; j < n_E; ++j) {
            polygon_t cellBorderPolygon = helperFunctionGetSquarePolygonFromMatrixCell(bottomBorder_N, leftBorder_E, i, j, minDimFootprint);


            std::deque<polygon_t> output;
            boost::geometry::intersection(boostFieldBorderPolygon, cellBorderPolygon, output);

            //ROS_INFO("(i,j) (%d, %d) SIZE = %.2fm^2", i, j, a);

            if (output.size() > 0) {
                // intersection found
//                double a = boost::geometry::area(*output.cbegin());
//                ROS_INFO("INTERSECTION AREA: %.2f in position (%d, %d) [NE]", a , i, j);
                matrix[i][j] = -1;
            } else {
                // not intersecting
                matrix[i][j] = -2;
//                ROS_INFO("Cell (%d, %d) [NE] NOT PART OF FIELD", i, j);
            }
        }
    }

}
