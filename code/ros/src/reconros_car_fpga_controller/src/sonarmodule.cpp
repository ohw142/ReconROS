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

#include "sonarmodule.h"

void sonarCallback(const reconros_car_msgs::CarSonar::ConstPtr& msg, SonarModule& module)
{
  module.processSonar(msg->sonar);
}

void stateCallback(const reconros_car_msgs::ScenarioState::ConstPtr& msg, SonarModule& module)
{
  module.processState(msg->paused);
}


SonarModule::SonarModule(ros::NodeHandle& node):
  Module(true),
  m_State_Pub(node.advertise<reconros_car_msgs::ScenarioStateRequest>("staterequest",10)),
  m_State_Sub(node.subscribe<reconros_car_msgs::ScenarioState>("state",10,boost::bind(stateCallback, _1, boost::ref(*this)))),
  m_Sonar_Sub(node.subscribe<reconros_car_msgs::CarSonar>("sonar",10,boost::bind(sonarCallback, _1, boost::ref(*this))))
{
}

SonarModule::~SonarModule()
{
}

void SonarModule::switchToSoftware()
{
  m_UseHardware = false;
}

void SonarModule::switchToHardware(int slot)
{
  char* bitstreams[3] = {"/usr/local/share/reconros/bitstreams/config_module_2_pblock_slot_0_partial.bit",
    "/usr/local/share/reconros/bitstreams/config_module_2_pblock_slot_1_partial.bit",
    "/usr/local/share/reconros/bitstreams/config_module_2_pblock_slot_2_partial.bit"};


  HWT_reconfigure(slot,bitstreams[slot]);
  m_UseHardware = true;
}

bool SonarModule::isUsingHardware()
{
  return m_UseHardware;
}

void SonarModule::processSonar(uint32_t sonarValue)
{
  if(isActive()) {
    uint32_t result;
    if(m_UseHardware) { // run on PL
      uint32_t data = sonarValue;
      result = HWT_process_module_2(&data);
    } else {
      if(sonarValue<60) {
        result = 1;
      } else if(sonarValue>80) {
        result = 2;
      } else {
        result = 0;
      }
    }
    if(result == 1 && !m_Paused) {
      reconros_car_msgs::ScenarioStateRequest msg;
      msg.property = "paused";
      msg.value = "1";
      m_State_Pub.publish(msg);
    } else if (result == 2 && m_Paused) {
      reconros_car_msgs::ScenarioStateRequest msg;
      msg.property = "paused";
      msg.value = "0";
      m_State_Pub.publish(msg);
    }
  }
}

void SonarModule::processState(bool paused)
{
  m_Paused = paused;
}

