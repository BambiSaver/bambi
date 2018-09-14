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

#include <bambi_msgs/TerrainDataProvider.h>
#include <bambi_msgs/TerrainDataProviderRequest.h>
#include <bambi_msgs/TerrainDataProviderResponse.h>

#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>
#include <algorithm>
#include <map>

#include <deque>
#include <utility>
#include <queue>

#include <boost/shared_ptr.hpp>
#include <boost/multi_array.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "../lib/spline/spline/src/main/cpp/BSpline.h"
#include "../lib/spline/spline/src/main/cpp/Bezier.h"
#include "advancedwavefrontsolver.h"

#include <stdio.h>
using namespace bambi::coverage_path_planner;

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
//typedef boost::geometry::model::d2::point_xy<double> point_t;
typedef boost::geometry::model::polygon<point_t> polygon_t;
typedef std::map<boost::tuple<int, int>, polygon_t> map_t;


CoveragePathPlannerNode::CoveragePathPlannerNode(const ros::NodeHandle &nodeHandle)
    : m_nodeHandle(nodeHandle) {

    m_publisherBambiPath = m_nodeHandle.advertise<bambi_msgs::Path>("path", 5, false);

    m_publisherRosNavPath = m_nodeHandle.advertise<nav_msgs::Path>("rviz_path", 5, false);



    m_subscriberTriggerPathGeneration = m_nodeHandle.subscribe("/bambi/mission_controller/trigger_path_generation", 10,
                                                               &CoveragePathPlannerNode::cb_trigger_path_generation, this);
}

void CoveragePathPlannerNode::spin() {
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

geodesy::UTMPoint getUTMCenterPointFromCellIndex(double start_N, double start_E, std::pair<int, int> index, float sensorFootprint, const geodesy::UTMPoint& bottomLeft) {
    std::pair<double,double> coordinates(
                start_N + index.first*sensorFootprint + sensorFootprint/2,
                start_E + index.second*sensorFootprint + sensorFootprint/2
                );
    return geodesy::UTMPoint(coordinates.second, coordinates.first, bottomLeft.zone, bottomLeft.band);
}


std::pair<int, int> getIndexOfMatrixByPoint(double _N, double _E, double start_N, double start_E, float sensorFootprint) {
    std::pair<int, int> t;
    t.first = std::floor((_N - start_N) / sensorFootprint)+1;
    t.second = std::floor((_E - start_E) / sensorFootprint)+1;
    return t;
}

bool checkIfToPutNextStep(boost::multi_array<int, 2>& matrix, int i, int j) {
    if (matrix[i][j] == -1) {
        // not initialized
        // MOVED TO CALLER FUNCTION: matrix[i][j] = currentStep + 1;
        return true;
    }/* else if (matrix[i][j] > 0) {
        // do nothing, because we passed already here, probably in a quicker way
    }*/
    return false;
}


short matchesTheNumber(const boost::multi_array<int, 2>& matrix, int i, int j, int theNumber) {
    if (matrix[i][j] == theNumber)
        return 1;
    return 0;
}

// returns the number of fields in the neighboorhood with the same number
short sameValuedNeighborFields(const boost::multi_array<int, 2>& matrix, int i, int j) {
    return matchesTheNumber(matrix, i, j+1, matrix[i][j])
            + matchesTheNumber(matrix, i+1, j+1, matrix[i][j])
            + matchesTheNumber(matrix, i+1, j, matrix[i][j])
            + matchesTheNumber(matrix, i+1, j-1, matrix[i][j])
            + matchesTheNumber(matrix, i, j-1, matrix[i][j])
            + matchesTheNumber(matrix, i-1, j-1, matrix[i][j])
            + matchesTheNumber(matrix, i-1, j, matrix[i][j])
            + matchesTheNumber(matrix, i-1, j+1, matrix[i][j]);
}

void printMatrix(const boost::multi_array<int, 2>& matrix, int n_N, int n_E)
{
    for (int i = n_N+1; i >= 0; --i) {
        std::ostringstream stringStream;
        for (int j = 0; j <= n_E+1; ++j) {
            stringStream << std::setw(4);
            stringStream << matrix[i][j];
        }
        ROS_INFO("%s", stringStream.str().c_str());
    }
}

void printDirectionMatrix(const boost::multi_array<std::string, 2>& matrix, int n_N, int n_E)
{
    for (int i = n_N+1; i >= 0; --i) {
        std::ostringstream stringStream;
        for (int j = 0; j <= n_E+1; ++j) {
            stringStream << std::setw(4);
            stringStream << matrix[i][j];
        }
        ROS_INFO("%s", stringStream.str().c_str());
    }
}

void printTerrainMatrix(const boost::multi_array<double, 2>& matrix, int n_N, int n_E)
{
    for (int i = n_N+1; i >= 0; --i) {
        std::ostringstream stringStream;
        for (int j = 0; j <= n_E+1; ++j) {
            stringStream << std::setw(5) << std::setprecision(4);
            stringStream << matrix[i][j];
        }
        ROS_INFO("%s", stringStream.str().c_str());
    }
}

const std::string getDirection(const std::pair<int,int>& from, const std::pair<int,int>& to) {
    if (to.first > from.first) {
        // went N
        if (to.second < from.second) {
            return "NW";
        } else if (to.second > from.second) {
            return "NE";
        } else {
            return "N";
        }
    } else if (to.first == from.first) {
        if (to.second < from.second) {
            return "W";
        } else if (to.second > from.second) {
            return "E";
        } else {
            // from == to (pair completely the same)
            return "***";
        }
    } else {
        if (to.second < from.second) {
            return "SW";
        } else if (to.second > from.second) {
            return "SE";
        } else {
            return "S";
        }
    }
}

void CoveragePathPlannerNode::cb_trigger_path_generation(const bambi_msgs::FieldCoverageInfo &fieldCoverageInfo) {

    // TODO: MAKE SAFE FOR EMPTY BOUNDARY ARRAY (coredumps for now)


    ROS_INFO("Got field coverage info, with a path of %d coordinates, flight heights (%.1f, %.1f) and sensor footprint %.1f x %.1f"
             , static_cast<int>(fieldCoverageInfo.field.boundary_path.size()), fieldCoverageInfo.relative_altitude_returning / 1E3
             , fieldCoverageInfo.relative_altitude_scanning, fieldCoverageInfo.thermal_camera_ground_footprint_width
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

    boost::multi_array<int, 2> matrix(boost::extents[n_N+2][n_E+2]);
    boost::multi_array<double, 2> terrainDataMatrix(boost::extents[n_N+2][n_E+2]);
    //boost::shared_ptr<map_t> map(new map_t());

    // make field larger to put in matrix (-2) on the borders for easier algorithm later
    bottomBorder_N -= minDimFootprint;
    leftBorder_E -= minDimFootprint;

    double fieldArea = boost::geometry::area(boostFieldBorderPolygon);
    ROS_INFO("FIELD IS %.2fm^2", std::abs<double>(fieldArea));


    for (int i = 0; i < n_N+2; ++i) {
        for (int j = 0; j < n_E+2; ++j) {

            if (i == 0 || j == 0 || i == n_N + 1 || j == n_E + 1) {
                matrix[i][j] = -2;
                continue;
            }

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
    printMatrix(matrix, n_N, n_E);


    /*************************************************
     *                TERRAIN DATA
     ************************************************/



    bambi_msgs::TerrainDataProvider service;
    bool serviceError = false;

    if (ros::service::waitForService("/bambi/terrain_data_provider", ros::Duration(2.0))){
        std::pair<int,int> index;

        for (index.first = 1; index.first <= n_N; ++index.first) {
            for (index.second = 1; index.second <= n_E; ++index.second) {
                if (matrix[index.first][index.second] == -1) {
                    auto utmPoint = getUTMCenterPointFromCellIndex(bottomBorder_N, leftBorder_E, index, minDimFootprint, bottomLeft);
                    geographic_msgs::GeoPoint geoPoint = geodesy::toMsg(utmPoint);
                    bambi_msgs::GeoPosition2D geoPos;
                    geoPos.latitude = geoPoint.latitude;
                    geoPos.longitude = geoPoint.longitude;
                    service.request.points.push_back(geoPos);
                }
            }
        }

        if (!ros::service::call("/bambi/terrain_data_provider" ,service)) {
            serviceError =  true;
        }
    } else {
        serviceError = true;
    }

    if (serviceError) {
        ROS_ERROR("/bambi/terrain_data_provider service not available, assuming a ground plane");
    } else {
        ROS_INFO("GOT TERRAIN DATA: %zd samples", service.response.terrainData.size());
    }

    // fill matrix
    std::pair<int,int> index;
    int k = 0;
    for (index.first = 1; index.first <= n_N; ++index.first) {
        for (index.second = 1; index.second <= n_E; ++index.second) {
            if (matrix[index.first][index.second] == -1) {
                if (!serviceError) {
                    terrainDataMatrix[index.first][index.second] = service.response.terrainData[k].altitudeInMeters;
                    ++k;
                } else {
                    terrainDataMatrix[index.first][index.second] = 0.0;
                }

            }
        }
    }

    printTerrainMatrix(terrainDataMatrix, n_N, n_E);



    /*************************************************
     *                TERRAIN END
     ************************************************/











    geographic_msgs::GeoPoint currentPosition = geodesy::toMsg(fieldCoverageInfo.current_position.geopos_2d.latitude, fieldCoverageInfo.current_position.geopos_2d.longitude);
    geodesy::UTMPoint currentPositionUTM(currentPosition);

    // TODO: We assume that current position is part of the field. --> may NOT be the case (?)
    // SOLUTION: check if the neighborfields are accessable (=-1) and not =(-2), and if not, choose a random accessable position to start

    //index = getIndexOfMatrixByPoint(currentPositionUTM.northing, currentPositionUTM.easting, bottomBorder_N, leftBorder_E, minDimFootprint);

    geographic_msgs::GeoPoint homePosition = geodesy::toMsg(fieldCoverageInfo.home_position.latitude, fieldCoverageInfo.home_position.longitude);
    geodesy::UTMPoint homePositionUTM(homePosition);

    index = getIndexOfMatrixByPoint(homePositionUTM.northing, homePositionUTM.easting, bottomBorder_N, leftBorder_E, minDimFootprint);

    ROS_INFO("Index of home position (%d, %d)", index.first, index.second);


    /*************************************************
     *         TEST ADVANCED WAVEFRONTSOLVER
     ************************************************/

//    boost::shared_ptr<AdvancedWaveFrontSolver> solver(new AdvancedWaveFrontSolver(n_E, n_N));

//    for (int i = 1; i <= n_N + 1; ++i) {
//        for (int j = 1; j <= n_E+ 1; ++j) {
//            if (matrix[i][j] == -1) {
//                solver->markCoverageCell(index_t(j,i));
//            }
//        }
//    }


//    solver->setStartCell(index_t(index.second + 1, index.first + 1));


//    std::deque<index_t> indexPath = solver->solveCoveragePath();

//    std::deque<geodesy::UTMPoint> trivialPointQueue;
//    for (auto posInverted : indexPath){

//        index_t position(posInverted.second, posInverted.first);
//        auto utmPoint = getUTMCenterPointFromCellIndex(bottomBorder_N, leftBorder_E, position, minDimFootprint, bottomLeft);
//        trivialPointQueue.push_back(utmPoint);
//    }

    /*************************************************
     *       TEST ADVANCED WAVEFRONTSOLVER END
     ************************************************/












    std::queue<std::pair<int, int>> cellQueue;
    cellQueue.push(index);
    matrix[index.first][index.second] = 0;

    std::pair<int,int> lastQueueElement;

    ROS_INFO("FILLING MATRIX");

    while (!cellQueue.empty()) {
        auto current = cellQueue.front();

        auto cellValue = matrix[current.first][current.second];
        float cellAltitude = terrainDataMatrix[current.first][current.second];

        for (int i = current.first - 1; i <= current.first +1; ++i) {
            for (int j = current.second - 1; j <= current.second +1; ++j) {
                if (i == current.first && j == current.second)
                    // skip same field
                    continue;
                if (checkIfToPutNextStep(matrix, i, j)) {
                    float slope = (terrainDataMatrix[i][j] - cellAltitude) / minDimFootprint;
                    //ROS_INFO("SLOPE: %.2f", slope);

//                    int penalty = 0;

//                    if (slope > 0.2f) {
//                        ++penalty;
//                        if (slope > 0.4f) {
//                            ++penalty;
//                            if (slope > 0.6f) {
//                                ++penalty;
//                            }
//                        }
//                    } else if (slope < 0.2f) {
//                        --penalty;
//                        if (slope < 0.4f) {
//                            --penalty;
//                            if (slope < 0.6f) {
//                                --penalty;
//                            }
//                        }
//                    }

                    matrix[i][j] = cellValue + 1 /*+ 4 + penalty*/;
                    cellQueue.push(std::pair<int,int>(i, j));
                }
            }
        }
        lastQueueElement = current;
        cellQueue.pop();
    }

    printMatrix(matrix, n_N, n_E);

    // look for starting position
    // starting from last queue element

    cellQueue.push(lastQueueElement);
    short lowestSameValuedNeighborFieldsNumber = 8;
    ROS_INFO("CHOOSING START POINT");
    auto myChosenStartPoint = lastQueueElement;

    std::set<std::pair<int, int>> checkedAlready;

    while (!cellQueue.empty()) {
        auto current = cellQueue.front();

        for (int i = current.first - 1; i <= current.first +1; ++i) {
            for (int j = current.second - 1; j <= current.second +1; ++j) {
                if (i == current.first && j == current.second)
                    // skip same field
                    continue;

                // consider only not-checked and same-valued fields
                if (checkedAlready.find(std::pair<int,int>(i, j)) == checkedAlready.end() &&
                        matrix[i][j] == matrix[current.first][current.second]) {
                    cellQueue.push(std::pair<int,int>(i, j));
                    short numSameValuedNeighborFieldsOfCurrent = sameValuedNeighborFields(matrix, i, j);

                    if (numSameValuedNeighborFieldsOfCurrent < lowestSameValuedNeighborFieldsNumber) {
                        myChosenStartPoint.first = i;
                        myChosenStartPoint.second = j;
                        lowestSameValuedNeighborFieldsNumber = numSameValuedNeighborFieldsOfCurrent;
                    }
                }


            }
        }

        checkedAlready.insert(current);
        cellQueue.pop();
    }

    //For test only override the starting position
//    myChosenStartPoint.first = 15;
//    myChosenStartPoint.second = 10;

    //

    ROS_INFO("START POINT = (%d, %d)", myChosenStartPoint.first, myChosenStartPoint.second);


    bool reachedEnd = false;
    auto currentPos = myChosenStartPoint;
    boost::multi_array<std::string, 2> directionMatrix(boost::extents[n_N+2][n_E+2]);

    bambi_msgs::Path varForPublishingBambi;
    nav_msgs::Path varForPublishingRos;

    std::deque<geodesy::UTMPoint> trivialPointQueue;

    while (!reachedEnd) {
        // assume reaching end
        reachedEnd = true;

        std::pair<int,int> bestChoice;
        int bestPriority = -1;

        for (int i = currentPos.first - 1; i <= currentPos.first + 1; ++i) {
            for (int j = currentPos.second - 1; j <= currentPos.second + 1; ++j) {
                if (i == currentPos.first && j == currentPos.second)
                    // skip same field
                    continue;


                if (matrix[i][j] >= 0) {
                    reachedEnd = false;

                    bool isDiagnoalField = ((i-currentPos.first) == (j -currentPos.second))
                                            || ((i-currentPos.first) == -(j -currentPos.second));

                    // the higher the better
                    int priority = matrix[i][j]*100 + (8 - sameValuedNeighborFields(matrix, i, j)) * 10 + (1- isDiagnoalField);
                    if (priority > bestPriority) {
                        bestPriority = priority;
                        bestChoice.first = i;
                        bestChoice.second = j;
                    }
                }
            }
        }

        if (!reachedEnd) {
            // I've been here
            matrix[currentPos.first][currentPos.second] = -3;

            auto utmPoint = getUTMCenterPointFromCellIndex(bottomBorder_N, leftBorder_E, currentPos, minDimFootprint, bottomLeft);
            // direction which ends at currentPosition

            trivialPointQueue.push_back(utmPoint);
            directionMatrix[currentPos.first][currentPos.second] = getDirection(currentPos, bestChoice);

            if (myChosenStartPoint == currentPos)
                directionMatrix[currentPos.first][currentPos.second]  = std::string("*") + directionMatrix[currentPos.first][currentPos.second] + std::string("*");

            currentPos = bestChoice;
        }
    }

    // mark last POS
    directionMatrix[currentPos.first][currentPos.second] = directionMatrix[currentPos.first][currentPos.second]  = std::string("#") + directionMatrix[currentPos.first][currentPos.second] + std::string("#");

    printDirectionMatrix(directionMatrix, n_N, n_E);

    printMatrix(matrix, n_N, n_E);





    //************************ COVERAGE FINISHED --> INTERPOLATING NOW *************************/


//    trivialPointQueue.push_front(currentPositionUTM);
//    geographic_msgs::GeoPoint homePosition = geodesy::toMsg(fieldCoverageInfo.home_position.latitude, fieldCoverageInfo.home_position.longitude);
//    geodesy::UTMPoint homePositionUTM(homePosition);

//    trivialPointQueue.push_back(homePositionUTM);

    std::vector<cppspline::Vector> xyzPath;
    xyzPath.reserve(trivialPointQueue.size() + 2);



    // use x y z like EAST, NORTH, UP (not NED)
    auto currentPosVec = cppspline::Vector(currentPositionUTM.easting,
                                           currentPositionUTM.northing,
                                           fieldCoverageInfo.current_position.altitude_over_ground /* METERS */
                                           );

    // currentPos 4 times to start from here (?)
    xyzPath.push_back(currentPosVec);
    xyzPath.push_back(currentPosVec);
    xyzPath.push_back(currentPosVec);
    xyzPath.push_back(currentPosVec);


    // generate x y z
    for (auto p : trivialPointQueue) {
        xyzPath.push_back(cppspline::Vector(p.easting,
                                            p.northing,
                                            fieldCoverageInfo.relative_altitude_scanning /* in meters*/
                                            ));
    }


    // use the last point to get back to home flight altitude

    auto lastPosVec = cppspline::Vector(xyzPath.back().x,
                                        xyzPath.back().y,
                                        fieldCoverageInfo.relative_altitude_returning /*METERS */
                                        );

    auto homePosVec = cppspline::Vector(homePositionUTM.easting,
                                        homePositionUTM.northing,
                                        fieldCoverageInfo.relative_altitude_returning /*METERS */
                                        );

    xyzPath.push_back(lastPosVec);
    xyzPath.push_back(lastPosVec);
    // 4 times to get home (?)
    xyzPath.push_back(homePosVec);
    xyzPath.push_back(homePosVec);
    xyzPath.push_back(homePosVec);
    xyzPath.push_back(homePosVec);

//TO MAKE MATLAB PLOT

    FILE * pFile;
      pFile = fopen ("/home/michael/CPP.txt","w+");
      if (pFile == NULL) {
              ROS_INFO("Opening the file failed.");
          }else
            ROS_INFO("Opening the file succeded.");
      for (auto p: xyzPath){
        fprintf (pFile, "%.4f, %.4f\n",p.x,p.y);
      }
      fclose (pFile);

//


    double pathLength = 0.0;
    for (auto it = xyzPath.cbegin() + 1; it != xyzPath.cend(); ++it) {
        pathLength += ((*it) - (*(it-1))).length();

    }

    // set required resolution
    static const double resolutionInMeters = 0.5;
    // limit to 100 steps
    int necessarySteps = std::min(static_cast<int>(std::ceil(pathLength / resolutionInMeters)), 100);

    ROS_INFO("With a resolution of %.2fm for the whole path with a length of %.2fm we need %i intermediate steps",
             resolutionInMeters, pathLength, necessarySteps);

    boost::shared_ptr<cppspline::Curve> curve(new cppspline::BSpline());

    curve->set_steps(necessarySteps);

    for (auto v : xyzPath) {
        curve->add_way_point(v);
    }


    //curve->


//    std::cout << "nodes: " << curve->node_count() << std::endl;
//    std::cout << "total length: " << curve->total_length() << std::endl;
//    for (int i = 0; i < curve->node_count(); ++i) {
//        std::cout << "node #" << i << ": " << curve->node(i).toString() << " (length so far: " << curve->length_from_starting_point(i) << ")" << std::endl;
//    }


    ROS_INFO("From a path with %d points, though CatmullRom we got an interpolation with %d points",
             static_cast<int>(xyzPath.size()), curve->node_count());


    //For MATLAB output
    pFile = fopen ("/home/michael/CPP-SPline.txt","w+");
    if (pFile == NULL) {
            ROS_INFO("Opening the file failed.");
        }else
          ROS_INFO("Opening the file succeded.");
    //
    for (int i = 0; i < curve->node_count(); ++i) {
        const cppspline::Vector& v = curve->node(i);
        geodesy::UTMPoint p(v.x, v.y, bottomLeft.zone, bottomLeft.band);
        geographic_msgs::GeoPoint geoPoint = geodesy::toMsg(p);

        // save for publishers
        bambi_msgs::GeoPositionWithRelativeAltitude bambiPoint;
        // use only scanning altitude for now TODO
        bambiPoint.altitude_over_ground = std::round(v.z);
        bambiPoint.geopos_2d.latitude = geoPoint.latitude;
        bambiPoint.geopos_2d.longitude = geoPoint.longitude;

        geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose.position.x = p.easting - leftBorder_E;
        poseStamped.pose.position.y = p.northing - topBorder_N;
        poseStamped.pose.position.z = v.z * 10E3;
        // TODO fix in rviz
        poseStamped.header.frame_id = "/map";

        varForPublishingBambi.geometric_path.push_back(bambiPoint);
        varForPublishingRos.poses.push_back(poseStamped);

        //FOR MATLAB OUTPUT
        fprintf (pFile, "%.4f, %.4f\n",v.x,v.y);

        //

    }
    //For MATLAB output
    fclose (pFile);
    //

    ROS_INFO("Publishing BAMBI and RVIZ Paths with %d and %d points",
             static_cast<int>(varForPublishingBambi.geometric_path.size()),
             static_cast<int>(varForPublishingRos.poses.size()));

    m_publisherBambiPath.publish(varForPublishingBambi);
    m_publisherRosNavPath.publish(varForPublishingRos);
}
