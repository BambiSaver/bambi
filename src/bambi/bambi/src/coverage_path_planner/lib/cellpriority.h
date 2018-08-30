/*
 * cellpriority.h
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
#ifndef CELLPRIORITY_H
#define CELLPRIORITY_H


#include "common.h"


namespace bambi {
namespace coverage_path_planner {

class AdvancedWaveFrontSolver;


class CellPriority
{
public:
    CellPriority(const AdvancedWaveFrontSolver& solver, float weight);

    virtual float getPriority(const index_t& currentCellIndex, const index_t& nextCellIndex, Direction heading = Direction::NONE) const = 0;

    float weight() const;
    void setWeight(float weight);

    virtual ~CellPriority() = default;


protected:
    // gives subclasses access to the solver matrix
    const float& cell(const index_t& index) const;


private:
    float m_weight;
    const AdvancedWaveFrontSolver& m_solver;
};

} // namespace coverage_path_planner
} // namespace bambi

#endif // CELLPRIORITY_H