/*
 * common.cpp
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

#include "common.h"

using namespace bambi::coverage_path_planner;

index_t bambi::coverage_path_planner::directionToNextCellIndex(const index_t &from, Direction direction)
{
    switch (direction) {
    case Direction::RIGHT:
        return index_t(from.first + 1, from.second);
    case Direction::UPRIGHT:
        return index_t(from.first + 1, from.second + 1);
    case Direction::UP:
        return index_t(from.first, from.second + 1);
    case Direction::UPLEFT:
        return index_t(from.first - 1, from.second + 1);
    case Direction::LEFT:
        return index_t(from.first - 1, from.second);
    case Direction::DOWNLEFT:
        return index_t(from.first - 1, from.second - 1);
    case Direction::DOWN:
        return index_t(from.first, from.second - 1);
    case Direction::DOWNRIGHT:
        return index_t(from.first + 1, from.second - 1);
    }
    return index_t(-1,-1);
}

Direction bambi::coverage_path_planner::nextCellIndexToDirection(const index_t &from, const index_t &to)
{
    if (to.first < from.first) {
        // LEFT
        if (to.second > from.second) {
            return Direction::UPLEFT;
        } else if (to.second == from.second) {
            return Direction::LEFT;
        } else {
            return Direction::DOWNLEFT;
        }
    } else if (to.first == from.first) {
        // UP OR DOWN
        if (to.second > from.second) {
            return Direction::UP;
        } else {
            return Direction::DOWN;
        }
    } else {
        // RIGHT
        if (to.second > from.second) {
            return Direction::UPRIGHT;
        } else if (to.second == from.second) {
            return Direction::RIGHT;
        } else {
            return Direction::DOWNRIGHT;
        }
    }
}

