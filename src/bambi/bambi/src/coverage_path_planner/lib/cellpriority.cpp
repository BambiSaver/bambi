/*
 * cellpriority.cpp
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
#include "cellpriority.h"
#include "advancedwavefrontsolver.h"

namespace bambi {
namespace coverage_path_planner {

CellPriority::CellPriority(const AdvancedWaveFrontSolver &solver, float weight)
    : m_solver(solver)
    , m_weight(weight)
{

}

float CellPriority::weight() const
{
    return m_weight;
}

void CellPriority::setWeight(float weight)
{
    m_weight = weight;
}

const float &CellPriority::cell(const index_t &index) const
{
    return m_solver.cell(index);
}


} // namespace coverage_path_planner
} // namespace bambi
