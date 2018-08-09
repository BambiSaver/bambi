/*
 * trajectory_generator.cpp
 *
 * Created: 2018/8/8 by Florian Mahlknecht <m@florian.world>
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
#include <ros/ros.h>
#include "lib/trajectorygeneratornode.h"


using namespace bambi::trajectory_generator;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle nh;
  TrajectoryGeneratorNode node(nh);
  ROS_INFO("Trajectory Generator STARTUP");
  node.spin();



  return 0;
}
