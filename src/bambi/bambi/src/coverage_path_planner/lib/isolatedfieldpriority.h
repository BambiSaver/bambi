/*
 * isolatedfieldpriority.h
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
#ifndef ISOLATEDFIELDPRIORITY_H
#define ISOLATEDFIELDPRIORITY_H

#include "cellpriority.h"

namespace bambi {
namespace coverage_path_planner {

class IsolatedFieldPriority : public bambi::coverage_path_planner::CellPriority
{
public:
    IsolatedFieldPriority(const AdvancedWaveFrontSolver& solver, float weight);


    virtual float getPriority(const index_t& currentCellIndex, const index_t& nextCellIndex, Direction heading = Direction::NONE) const;
    bool almostEquals(const index_t& c1, const index_t& c2) const;

private:
    static const float EPSILON_DISCRETIZE;
    short getNumOfSameValuedNeighborFields(const index_t& index) const;
};

} // namespace coverage_path_planner
} // namespace bambi

#endif // ISOLATEDFIELDPRIORITY_H