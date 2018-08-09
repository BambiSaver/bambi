/*
 * utilities.h
 *
 * Created: 2018/8/9 by Florian Mahlknecht <m@florian.world>
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
#ifndef UTILITIES_H
#define UTILITIES_H

#include <mavros_msgs/ExtendedState.h>


namespace bambi {

class Utilities
{
public:
  
  static const std::map<mavros_msgs::ExtendedState::_landed_state_type, const char *> landedStateToStringMap;
};

} // namespace bambi

#endif // UTILITIES_H