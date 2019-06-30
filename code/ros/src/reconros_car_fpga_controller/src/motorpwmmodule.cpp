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

#include "motorpwmmodule.h"

void steeringCallback(const reconros_car_msgs::CarSteering::ConstPtr& msg, MotorPwmModule& module)
{
  module.processSteering(msg->steering);
}

void throttleCallback(const reconros_car_msgs::CarThrottle::ConstPtr& msg, MotorPwmModule& module)
{
  module.processThrottle(msg->throttle);
}

MotorPwmModule::MotorPwmModule(ros::NodeHandle& node):
  Module(false),
  m_Steering_Sub(node.subscribe<reconros_car_msgs::CarSteering>("steering",10,boost::bind(steeringCallback,_1,boost::ref(*this)))),
  m_Throttle_Sub(node.subscribe<reconros_car_msgs::CarThrottle>("throttle",10,boost::bind(throttleCallback,_1,boost::ref(*this))))
{
}

MotorPwmModule::~MotorPwmModule()
{
}

void MotorPwmModule::switchToHardware(int slot)
{
  // Check if it is in the correct slot
  if(slot!=3)
    return;
  char* bitstream = "/usr/local/share/reconros/bitstreams/config_module_4_pblock_slot_3_partial.bit";

  HWT_reconfigure(slot,bitstream);
  // Send default values
  uint32_t data[2] = {0,90};
  HWT_process_module_4(data);
  data[0] = 1;
  HWT_process_module_4(data);
}

void MotorPwmModule::processSteering(int8_t steeringValue)
{
  uint32_t data[2] = {0,(uint32_t)((int32_t)steeringValue+90)};
  HWT_process_module_4(data);
}

void MotorPwmModule::processThrottle(int8_t throttleValue)
{
  uint32_t data[2] = {1,(uint32_t)((int32_t)throttleValue+90)};
  HWT_process_module_4(data);
}

