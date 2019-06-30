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

#include "ledstopmodule.h"

void ledCallback(const reconros_car_msgs::CarLEDArray::ConstPtr& msg, LedStopModule& module)
{
  module.processLed(msg->leds);
}

void stateCallback(const reconros_car_msgs::ScenarioState::ConstPtr& msg, LedStopModule& module)
{
  module.processState(msg->running,msg->paused);
}

LedStopModule::LedStopModule(ros::NodeHandle& node):
  Module(true),
  m_State_Pub(node.advertise<reconros_car_msgs::ScenarioStateRequest>("staterequest",10)),
  m_Steering_Pub(node.advertise<reconros_car_msgs::CarSteering>("steeringrequest",10)),
  m_State_Sub(node.subscribe<reconros_car_msgs::ScenarioState>("state",10,boost::bind(stateCallback, _1, boost::ref(*this)))),
  m_Led_Sub(node.subscribe<reconros_car_msgs::CarLEDArray>("ledarray",10,boost::bind(ledCallback, _1, boost::ref(*this))))
{
}

LedStopModule::~LedStopModule()
{
}

void LedStopModule::switchToSoftware()
{
  m_UseHardware = false;
}

void LedStopModule::switchToHardware(int slot)
{
  char* bitstreams[3] = {"/usr/local/share/reconros/bitstreams/config_module_1_pblock_slot_0_partial.bit",
      "/usr/local/share/reconros/bitstreams/config_module_1_pblock_slot_1_partial.bit",
      "/usr/local/share/reconros/bitstreams/config_module_1_pblock_slot_2_partial.bit"};

  HWT_reconfigure(slot,bitstreams[slot]);
  m_UseHardware = true;
}

bool LedStopModule::isUsingHardware()
{
  return m_UseHardware;
}

void LedStopModule::processLed(uint32_t ledValue)
{
  if(isActive()) {
    uint32_t result;
    if(m_UseHardware) { // run on PL
      uint32_t data = ledValue;
      result = HWT_process_module_1(&data);
    } else {
      char leds = ledValue;
      /*char bits = 0;
      while(leds) {
        leds &= leds-1;
        bits++;
      }   
      result = (bits>7)?1:0;*/
      char start=-1;
      char stop=-1;
      for(int i = 0; i<8; i++) {
        if(leds&(1<<i)){
          start = i;  
          break;
        }
      }
      for(int i = 7; i>=0; i--) {
        if(leds&(1<<i)){
          stop = i;   
          break;
        }
      }
        
      char middle=(start+stop+1)>>1;
      if(middle<0){
        result=0;
      }else if(middle<1){
        result=1;
      }else if(middle<3){
        result=2;
      }else if(middle<5){
        result=3;
      }else if(middle<7){
        result=4;
      }else{
        result=5;
      }

    }
    
    if (m_Running && !m_Paused){
      if(result==0){
        reconros_car_msgs::ScenarioStateRequest msg;
        msg.property = "paused";
        msg.value = "1";
        m_State_Pub.publish(msg);
      }else{
        int8_t steeringValues[5] = {18, 8, 0, -8, -18};
        reconros_car_msgs::CarSteering msg;
        msg.steering = steeringValues[result-1];   
        m_Steering_Pub.publish(msg);
      }
    }

  }
}

void LedStopModule::processState(bool running, bool paused)
{
  m_Running = running;
  m_Paused = paused;
}
