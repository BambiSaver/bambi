/*
 * common.h
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
#ifndef COMMON_H
#define COMMON_H

#include <utility>
#include <set>

namespace bambi {
namespace coverage_path_planner {

enum class Direction {
    NONE,
    RIGHT,
    UPRIGHT,
    UP,
    UPLEFT,
    LEFT,
    DOWNLEFT,
    DOWN,
    DOWNRIGHT
};


typedef std::pair<int,int> index_t;

static const std::set<Direction> DIAGONAL_DIRECTIONS  = {
    Direction::UPRIGHT,
    Direction::UPLEFT,
    Direction::DOWNLEFT,
    Direction::DOWNRIGHT
};

index_t directionToNextCellIndex(const index_t& from, Direction direction);
Direction nextCellIndexToDirection(const index_t& from, const index_t& to);

}
}

#endif // COMMON_H
