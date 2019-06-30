/*
 * ReconROS
 * Copyright (C) 2019  Andreas Krakau, Felix Paul Jentzsch
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <boost/bind.hpp>

#include "ros/ros.h"
#include <ros/console.h>

#include "reconros_car_msgs/CarConfiguration.h"
#include "configuration.h"

#include <reconros_p2.h>

void configCallback(const reconros_car_msgs::CarConfiguration::ConstPtr& msg, Configuration& config){
  config.setModuleConfig(msg->module,msg->state,msg->slot);
}


int main(int argc, char **argv) {
  ROS_DEBUG("Starting ROS node...");
  ros::init(argc, argv, "image_controller");
  ros::NodeHandle n;
  ROS_DEBUG("Initializing hardware...");
  HWT_init();
  ROS_DEBUG("Loading bitstreams...");
  Configuration config(n);
  
  ros::Subscriber config_sub = n.subscribe<reconros_car_msgs::CarConfiguration>("configuration2",10, boost::bind(configCallback, _1, boost::ref(config)));

  ROS_DEBUG("Running...");
  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();
  ros::waitForShutdown();
  HWT_shutdown();
  return 0;
}
