/*
 * advancedwavefrontsolver.cpp
 *
 * Created: 2018/8/21 by Florian Mahlknecht <m@florian.world>
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
#include "advancedwavefrontsolver.h"

#include <cmath>
#include <queue>
#include <ros/ros.h> // DEBUG OUTPUT... (TODO find better solution)
#include <sstream>
#include <limits>
#include <vector>
#include <functional> // std::greater
#include "distancepriority.h"
#include "isolatedfieldpriority.h"

using namespace bambi::coverage_path_planner;

const float AdvancedWaveFrontSolver::EPSILON = .0001f;
const float AdvancedWaveFrontSolver::MARK_OBSTACLE = -2.f;
const float AdvancedWaveFrontSolver::MARK_COVERAGE = -1.f;


float AdvancedWaveFrontSolver::standardCostFunction(const index_t &from, Direction direction)
{
    if (DIAGONAL_DIRECTIONS.find(direction) == DIAGONAL_DIRECTIONS.end())
        return 1.f;
    return 1.414213562f;
}


void AdvancedWaveFrontSolver::printMatrix()
{
    ROS_INFO("################# AdvancedWaveFrontSolver MATRIX #######################");
    for (int i = m_m+1; i >= 0; --i) {
        std::ostringstream stringStream;
        for (int j = 0; j <= m_n+1; ++j) {
            stringStream << std::setw(5) << std::setprecision(3);
            stringStream << m_matrix[j][i];
        }
        ROS_INFO("%s", stringStream.str().c_str());
    }
    ROS_INFO("############### AdvancedWaveFrontSolver MATRIX END #####################");
}

index_t AdvancedWaveFrontSolver::propagateAndGetFarestCell()
{
    std::queue<index_t> cellQueue;
    // initialize to something useful, since we need a comparison value
    index_t farestCellIndex = m_startIndex;


    cellQueue.push(m_startIndex);
    cell(m_startIndex) = 0.f;


    while (!cellQueue.empty()) {
        index_t currentCellIndex = cellQueue.front();
        float currentCellValue = cell(currentCellIndex);

        // loop all neighborcells
        index_t index;
        for (index.first = currentCellIndex.first - 1; index.first <= currentCellIndex.first + 1; ++index.first) {
            for (index.second = currentCellIndex.second -1; index.second <= currentCellIndex.second + 1; ++index.second) {
                // skip same cell
                if (index == currentCellIndex)
                    continue;


                auto direction = nextCellIndexToDirection(currentCellIndex, index);
                float cost = m_costFunction(currentCellIndex, direction);
                float proposedNewValue = currentCellValue + cost;

                // if the proposedNewValue is smaller, we've just found a less costy way to get here
                if (proposedNewValue < cell(index) || equals(cell(index), MARK_COVERAGE)) {
                    cell(index) = proposedNewValue;
                    cellQueue.push(index);
                }

                if (cell(farestCellIndex) < cell(index)) {
                    farestCellIndex = index;
                }
            }
        }
        cellQueue.pop();
    }
    return farestCellIndex;
}

index_t AdvancedWaveFrontSolver::chooseBestStartPoint(const index_t &farestCellToCover)
{
    // choose the most isolated start point --> the one with the lowest number of same valued neighbor fields TODO is this necessary?
    // TODO why don't we just start the algorithm from the 0 cell?
    std::queue<index_t> cellQueue;
    cellQueue.push(farestCellToCover);
    float bestIsolatedFieldPriority = -1.f;
    auto chosenStartPoint = farestCellToCover;

    std::set<index_t> alreadyCheckedIndicies;

    while (!cellQueue.empty()) {
        auto currentCellIndex = cellQueue.front();

        // loop all neighborcells
        index_t index;
        for (index.first = currentCellIndex.first - 1; index.first <= currentCellIndex.first + 1; ++index.first) {
            for (index.second = currentCellIndex.second -1; index.second <= currentCellIndex.second + 1; ++index.second) {
                // skip same cell
                if (index == currentCellIndex)
                    continue;

                bool isChecked = alreadyCheckedIndicies.find(index) != alreadyCheckedIndicies.end();

                if (!isChecked && m_isolatedFieldPriority->almostEquals(currentCellIndex, index)) {
                    cellQueue.push(index);

                    float currentIsolatedFieldPriority = m_isolatedFieldPriority->getPriority(index_t(), index);

                    if (currentIsolatedFieldPriority > bestIsolatedFieldPriority) {
                        bestIsolatedFieldPriority = currentIsolatedFieldPriority;
                        chosenStartPoint = index;
                    }

                    /*
                    short numOfSameValuedNeighborFields = getNumOfSameValuedNeighborFields(index);

                    if (numOfSameValuedNeighborFields < lowestNumOfSameValuedNeighborFields) {
                        lowestNumOfSameValuedNeighborFields = numOfSameValuedNeighborFields;
                        chosenStartPoint = index;
                    }*/
                }
            }
        }
        alreadyCheckedIndicies.insert(currentCellIndex);
        cellQueue.pop();
    }

    return chosenStartPoint;
}

float &AdvancedWaveFrontSolver::cell(const index_t &index)
{
    return m_matrix[index.first][index.second];
}

const float &AdvancedWaveFrontSolver::cell(const index_t &index) const
{
    return m_matrix[index.first][index.second];
}


const bool AdvancedWaveFrontSolver::equals(float a, float b)
{
    return std::abs(a-b) < EPSILON;
}


AdvancedWaveFrontSolver::AdvancedWaveFrontSolver(int n, int m, costFunction cost) :
    m_matrix(boost::extents[n+2][m+2]),
    m_n(n),
    m_m(m),
    m_costFunction(cost),
    m_isolatedFieldPriority(new IsolatedFieldPriority(*this, 1.f)),
    m_cellPriorities({
                     boost::shared_ptr<CellPriority>(new DistancePriority(*this, 1.f)),
                     m_isolatedFieldPriority
                     })
{
    clear();
}

void AdvancedWaveFrontSolver::clear()
{
    m_coverageCellCount = 0;
    for (int i = 0; i <= m_n+1; ++i) {
        for (int j = 0; j <= m_m+1; ++j) {
            m_matrix[i][j] = MARK_OBSTACLE;
        }
    }
}

void AdvancedWaveFrontSolver::markCoverageCell(const index_t &index)
{
    if (!equals(cell(index), MARK_COVERAGE)) {
        ++m_coverageCellCount;
        cell(index) = MARK_COVERAGE;
    }
}

void AdvancedWaveFrontSolver::setStartCell(const index_t &start)
{
    m_startIndex = start;
}

std::deque<index_t> AdvancedWaveFrontSolver::solveCoveragePath()
{
    auto lastCell = propagateAndGetFarestCell();
    printMatrix();
    ROS_INFO("FAREST CELL = (%d, %d)", lastCell.first, lastCell.second);



    /*


    std::deque<index_t> path;
    auto currentCellIndex = m_startIndex;
    float currentStep = -3.f;

    cell(currentCellIndex) = currentStep;
    int coveredCells = 1;


    while (coveredCells < m_coverageCellCount) {
        // assume to have to go back
        bool haveToGoBack = true;

        index_t bestChoice;
        float lowestCost =  std::numeric_limits<float>::max();
        // init to 0.0f CELL
        index_t lowestNeighborToGoBack = m_startIndex;

        // loop all neighborcells
        index_t index;
        for (index.first = currentCellIndex.first - 1; index.first <= currentCellIndex.first + 1; ++index.first) {
            for (index.second = currentCellIndex.second -1; index.second <= currentCellIndex.second + 1; ++index.second) {
                if (index == currentCellIndex)
                    continue;


                if (cell(index) >= 0.f) {
                    haveToGoBack = false;

                    float cost = cell(index)*10.f + getNumOfSameValuedNeighborFields(index);

                    if (cost < lowestCost) {
                        lowestCost = cost;
                        bestChoice = index;
                    }
                } else if (cell(currentCellIndex) < cell(index) && cell(index) < cell(lowestNeighborToGoBack)) {
                    ROS_INFO("From %.1f I choose to rather go back to %.1f", cell(currentCellIndex), cell(index));
                    lowestNeighborToGoBack = index;
                }
            }
        }

        index_t nextStep = haveToGoBack ? lowestNeighborToGoBack : bestChoice;


        currentStep -= 1.f;
        if (!haveToGoBack) {
            cell(nextStep) = currentStep;
            ++coveredCells;
        }

        path.push_back(nextStep);
        currentCellIndex = nextStep;

        if (!haveToGoBack) {
            ROS_INFO("Going forward to (%d, %d)", currentCellIndex.first, currentCellIndex.second);
            //++coveredCells;
            //ROS_INFO("Covered %d out of %d cells", coveredCells, m_coverageCellCount);
        } else {
            ROS_INFO("Going back to (%d, %d)", currentCellIndex.first, currentCellIndex.second);
        }


        if (currentStep < -100.f) {
            break;
        }
    }





    printMatrix();


*/




    auto startPoint = chooseBestStartPoint(lastCell);
    ROS_INFO("END POINT = (%d, %d)", startPoint.first, startPoint.second);




    std::deque<index_t> path;
    auto currentCellIndex = startPoint;
    float currentStep = -3.f;

    //cell(currentCellIndex) = currentStep;
    int coveredCells = 1;

    // TODO prefer to start in which direction?
    Direction lastDirection = Direction::UP;

    while (coveredCells < m_coverageCellCount) {
        // assume reaching end
        bool haveToGoBack = true;


        std::priority_queue<std::pair<float, index_t>> priorityQueue;
//        // less is better
//        std::priority_queue<std::pair<int, index_t>,
//                std::vector<std::pair<int, index_t>>,
//                std::greater<std::pair<int, index_t>>> turningBadnessQueue;
//        // less is better
//        std::priority_queue<std::pair<short, index_t>,
//                std::vector<std::pair<short, index_t>>,
//                std::greater<std::pair<int, index_t>>> sameValuedNeighborQueue;


//        index_t bestChoice;
//        float bestPriority = -1.f;
        //short numOfSameValuedNeighbors = 8;
//        Direction newDirection;
        // init to 0.0f CELL
        index_t lowestNeighborToGoBack = m_startIndex;



        // loop all neighborcells
        index_t index;
        for (index.first = currentCellIndex.first - 1; index.first <= currentCellIndex.first + 1; ++index.first) {
            for (index.second = currentCellIndex.second -1; index.second <= currentCellIndex.second + 1; ++index.second) {
                if (index == currentCellIndex)
                    continue;


                if (cell(index) >= 0.f) {
                    haveToGoBack = false;



                    float newPriority = 0.f;
                    for (auto priority : m_cellPriorities) {
                        newPriority += priority->weight() * priority->getPriority(currentCellIndex, index, lastDirection);
                    }

//                    ROS_INFO("Accessing (%d, %d) -> (%d,%d): %.2f", currentCellIndex.first, currentCellIndex.second, index.first, index.second, newPriority);

                    priorityQueue.push(std::pair<float,index_t>(newPriority, index));

                    /*if (newPriority > bestPriority) {
                        bestPriority = newPriority;
                        bestChoice = index;
                        newDirection = nextCellIndexToDirection(currentCellIndex, index);
                    }*/

                    //float priority =  cell(index);

                    /*

                    if (almostEquals(priority, bestPriority)) {
                        // if they are almost equal try to not change direction
                        if (newDirection != lastDirection) {
                            // if we change direction anyways, go to the more isolated field
                            short neighbors = getNumOfSameValuedNeighborFields(index);
                            if (neighbors < numOfSameValuedNeighbors) {
                                numOfSameValuedNeighbors = neighbors;
                                bestChoice = index;
                                bestPriority = priority;
                                newDirection = nextCellIndexToDirection(currentCellIndex, index);
                            }
                        }

                    } else if (priority > bestPriority) {
                        numOfSameValuedNeighbors = getNumOfSameValuedNeighborFields(index);
                        bestPriority = priority;
                        bestChoice = index;
                        newDirection = nextCellIndexToDirection(currentCellIndex, index);
                    }*/

                    /*
                    if (priority > bestPriority) {
                        bestPriority = priority;
                        bestChoice = index;
                    }*/
                } else if (cell(index) < cell(lowestNeighborToGoBack)) {
                    // TODO don't go back to the lowest neighbor, but take the fastest path to come to
                    lowestNeighborToGoBack = index;
                }
            }
        }

        index_t nextStep; // = haveToGoBack ? lowestNeighborToGoBack : bestChoice;

        if (haveToGoBack) {
            nextStep = lowestNeighborToGoBack;
        } else {
            nextStep = priorityQueue.top().second;
        }


        cell(currentCellIndex) = currentStep;
        currentStep -= 1.f;
        lastDirection = nextCellIndexToDirection(currentCellIndex, nextStep);
        path.push_front(nextStep);
        currentCellIndex = nextStep;

        if (!haveToGoBack) {
            ++coveredCells;
        }
    }


    printMatrix();




    return path;
}

