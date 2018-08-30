/*
 * distancepriority.cpp
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
#include "distancepriority.h"
#include "advancedwavefrontsolver.h"

using namespace bambi::coverage_path_planner;

DistancePriority::DistancePriority(const AdvancedWaveFrontSolver &solver, float weight)
    : CellPriority(solver, weight)
{
}

float DistancePriority::getPriority(const index_t &currentCellIndex, const index_t &nextCellIndex, Direction heading) const
{
    // generates values between 0 and 1, as long as the maximum distance is sqrt(3) (read by the potential field in the matrix),
    // i.e. a DIAGONAL FIELD with 100% inclination
    // N.B. generally also > 100% would be possible, in that case we're just gonna give back something > 1.0
    return (OFFSET + cell(nextCellIndex) - cell(currentCellIndex))*SCALING;
}

