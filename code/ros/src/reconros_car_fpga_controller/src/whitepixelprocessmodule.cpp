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

#include "whitepixelprocessmodule.h"

void whitePixelsProcessCallback(const reconros_car_msgs::ImageWhiteFilter::ConstPtr& msg, WhitePixelProcessModule& module)
{
  module.processData(msg->bins);
}

void stateCallback(const reconros_car_msgs::ScenarioState::ConstPtr& msg, WhitePixelProcessModule& module)
{
  module.processState(msg->running);
}

WhitePixelProcessModule::WhitePixelProcessModule(ros::NodeHandle& node) :
  Module(true),
  m_Steering_Pub(node.advertise<reconros_car_msgs::CarSteering>("steeringrequest",10)),
  m_State_Sub(node.subscribe<reconros_car_msgs::ScenarioState>("state",10,boost::bind(stateCallback, _1, boost::ref(*this)))),
  m_White_Pixels_Sub(node.subscribe<reconros_car_msgs::ImageWhiteFilter>("white_pixels",10,boost::bind(whitePixelsProcessCallback, _1, boost::ref(*this))))

{

}

WhitePixelProcessModule::~WhitePixelProcessModule()
{
}

void WhitePixelProcessModule::switchToSoftware() {
  m_UseHardware = false;
}

void WhitePixelProcessModule::switchToHardware(int slot) {
  char* bitstreams[3] = {"/usr/local/share/reconros/bitstreams/config_module_3_pblock_slot_0_partial.bit",
      "/usr/local/share/reconros/bitstreams/config_module_3_pblock_slot_1_partial.bit",
      "/usr/local/share/reconros/bitstreams/config_module_3_pblock_slot_2_partial.bit"};
  HWT_reconfigure(slot,bitstreams[slot]);
  m_UseHardware = true;
}

bool WhitePixelProcessModule::isUsingHardware()
{
  return m_UseHardware;
}

void WhitePixelProcessModule::processData(std::vector<uint32_t> bins) {
  if(isActive()) {
    uint32_t result;
    if(m_UseHardware) { // run on PL
      if(bins.size()!=15){
        result = 7;
      } else {
        uint32_t data[15];
        for(int i = 0; i<15;i++){
          data[i] = bins[i];
        }   
        result = HWT_process_module_3(data);
      }   
    } else {
      uint32_t maxValue=0;
      result = 0;
      for(int i = 0; i<bins.size();i++){
        if(maxValue <bins[i]){
          maxValue = bins[i];
          result = i;
        }   
      }   
    }
    int8_t steeringValues[15] = { -25, -20, -15, -10, -7, -4, -2, 0, 2, 4, 7, 10, 15, 20, 25};
    if (m_Running){
      reconros_car_msgs::CarSteering msg;
      msg.steering = steeringValues[result];
      m_Steering_Pub.publish(msg);
    }
  }
}

void WhitePixelProcessModule::processState(bool running)
{
  m_Running = running;
}
