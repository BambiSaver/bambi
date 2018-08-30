/*
 * advancedwavefrontsolver.h
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
#ifndef ADVANCEDWAVEFRONTSOLVER_H
#define ADVANCEDWAVEFRONTSOLVER_H

#include <boost/multi_array.hpp>
#include <boost/shared_ptr.hpp>
#include <deque>
#include <vector>

#include "common.h"
#include "cellpriority.h"

namespace bambi {
namespace coverage_path_planner {

class IsolatedFieldPriority;

class AdvancedWaveFrontSolver
{
public:
    friend class bambi::coverage_path_planner::CellPriority;

    typedef float (*costFunction)(const index_t& from, Direction direction);

    AdvancedWaveFrontSolver(int n, int m, costFunction cost = standardCostFunction);

    /**
     * Clears the internal structures, to be able to calculate another path
     * using the same matrix dimensions.
     * @brief clear
     */
    void clear();
    /**
     * @brief markCoverageCell
     * @param index from 1 <= N, 1 <= M !!!
     */
    void markCoverageCell(const index_t& index);
    /**
     * @brief setStartCell
     * @param start index from 1 <= N, 1 <= M !!!
     */
    void setStartCell(const index_t& start);

    std::deque<index_t> solveCoveragePath();


    static float standardCostFunction(const index_t& from, Direction direction);


private:
    boost::multi_array<float, 2> m_matrix;
    int m_n;
    int m_m;
    index_t m_startIndex;
    costFunction m_costFunction;
    int m_coverageCellCount;
    boost::shared_ptr<IsolatedFieldPriority> m_isolatedFieldPriority;
    std::vector<boost::shared_ptr<CellPriority>> m_cellPriorities;

    void printMatrix();
    float& cell(const index_t& index);
    const float& cell(const index_t& index) const;
    index_t propagateAndGetFarestCell();
    index_t chooseBestStartPoint(const index_t& farestCellToCover);

    static const float EPSILON;
    static const float MARK_OBSTACLE;
    static const float MARK_COVERAGE;
    static const bool equals(float a, float b);
};

} // namespace coverage_path_planner
} // namespace bambi

#endif // ADVANCEDWAVEFRONTSOLVER_H