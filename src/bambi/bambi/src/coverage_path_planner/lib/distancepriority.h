/*
 * distancepriority.h
 *
 * Created: 2018/8/22 by Florian Mahlknecht <m@florian.world>
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
#ifndef DISTANCEPRIORITY_H
#define DISTANCEPRIORITY_H

#include "cellpriority.h"


namespace bambi {
namespace coverage_path_planner {

class DistancePriority : public CellPriority
{
public:
    DistancePriority(const AdvancedWaveFrontSolver& solver, float weight);

    virtual float getPriority(const index_t& currentCellIndex, const index_t& nextCellIndex, Direction heading = Direction::NONE) const;

    virtual ~DistancePriority() = default;

private:
    // sqrt(3)
    static float constexpr OFFSET = 1.732050808f;
    // 1 / (2*sqrt(3))
    static float constexpr SCALING = 0.288675135f;
};

} // namespace coverage_path_planner
} // namespace bambi

#endif // DISTANCEPRIORITY_H