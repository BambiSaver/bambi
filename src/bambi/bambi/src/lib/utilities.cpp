/*
 * utilities.cpp
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
#include "utilities.h"

namespace bambi {

const std::map<mavros_msgs::ExtendedState::_landed_state_type, const char *>  Utilities::landedStateToStringMap = {
  { mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR, "LANDED_STATE_IN_AIR" },
  { mavros_msgs::ExtendedState::LANDED_STATE_LANDING, "LANDED_STATE_LANDING" },
  { mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND, "LANDED_STATE_ON_GROUND" },
  { mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF, "LANDED_STATE_TAKEOFF" },
  { mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED, "LANDED_STATE_UNDEFINED" },
};


} // namespace bambi
