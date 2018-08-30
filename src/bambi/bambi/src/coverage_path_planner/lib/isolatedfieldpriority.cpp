/*
 * isolatedfieldpriority.cpp
 *
 * Created: 2018/8/25 by Florian Mahlknecht <m@florian.world>
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
#include "isolatedfieldpriority.h"
#include <cmath> // std::abs()

using namespace bambi::coverage_path_planner;


const float IsolatedFieldPriority::EPSILON_DISCRETIZE = .5f;

IsolatedFieldPriority::IsolatedFieldPriority(const AdvancedWaveFrontSolver &solver, float weight)
    : CellPriority(solver, weight)
{

}

float IsolatedFieldPriority::getPriority(const index_t &currentCellIndex, const index_t &nextCellIndex, Direction heading) const
{
    return (8.f - getNumOfSameValuedNeighborFields(nextCellIndex)) / 8.f;
}


short IsolatedFieldPriority::getNumOfSameValuedNeighborFields(const index_t &currentCellIndex) const
{
    short numberOfSameValuedNeighborFields = 0;
    // loop all neighborcells
    index_t index;
    for (index.first = currentCellIndex.first - 1; index.first <= currentCellIndex.first + 1; ++index.first) {
        for (index.second = currentCellIndex.second -1; index.second <= currentCellIndex.second + 1; ++index.second) {
            if (index == currentCellIndex)
                continue;
            if (almostEquals(currentCellIndex, index)) {
                ++numberOfSameValuedNeighborFields;
            }
        }
    }
    return numberOfSameValuedNeighborFields;
}


bool IsolatedFieldPriority::almostEquals(const index_t& c1, const index_t& c2) const
{
    return std::abs(cell(c1)-cell(c2)) < EPSILON_DISCRETIZE;
}
