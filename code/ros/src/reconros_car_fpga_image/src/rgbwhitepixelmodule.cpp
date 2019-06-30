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

#include "rgbwhitepixelmodule.h"

RgbWhitePixelModule::RgbWhitePixelModule(ros::NodeHandle& node):
  Module(false),
  m_WhitePixel_Pub(node.advertise<reconros_car_msgs::ImageWhiteFilter>("white_pixels",10))
{
}

RgbWhitePixelModule::~RgbWhitePixelModule()
{
  if(m_ProcessThread.joinable()){
    m_ProcessThread.join();
  }
}

void RgbWhitePixelModule::switchToHardware(int slot)
{
  char* bitstreams[1] = {"/usr/local/share/reconros/bitstreams/config_module_2_pblock_slot_0_partial.bit"};


  HWT_reconfigure(slot,bitstreams[slot]);
}

void RgbWhitePixelModule::setIsActive(bool value)
{
  if(value!=isActive()) {
    Module::setIsActive(value);
    if(value){
      startProcess();
    }
  }
}

bool RgbWhitePixelModule::isUsingHardware()
{
  return true;
}

void RgbWhitePixelModule::startProcess()
{
  m_ProcessThread = std::thread(&RgbWhitePixelModule::process, this);
}

void RgbWhitePixelModule::process()
{
  uint32_t data[31];
  data[0] = 150;  //default threshold
  for(int i=1; i<15; i++)
  {
        data[i] = 85*(i-1); //default spacing of column sectors
  }
  data[15]=1278;

  HWT_start_module_2(data); // begin processing frames
  while(ros::ok() && isActive()){
    HWT_waiton_module_2();

    reconros_car_msgs::ImageWhiteFilter msg;
    msg.bins.clear();
    for(int i = 0; i<15; i++)
    {
      msg.bins.push_back(data[16+i]);
    } 
    m_WhitePixel_Pub.publish(msg);
  }
}
