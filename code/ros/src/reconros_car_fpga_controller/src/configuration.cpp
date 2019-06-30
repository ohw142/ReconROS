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

#include "configuration.h"


Configuration::Configuration(ros::NodeHandle& node):
  m_SonarModule(new SonarModule(node)),
  m_LedModule(new LedStopModule(node)),
  m_WhitePixelProcessModule(new WhitePixelProcessModule(node)),
  m_MotorPwmModule(new MotorPwmModule(node))
{
  for(int i=0;i<SLOT_COUNT;i++) {
    m_Slots[i]=-1;
  }
  // MotorPwmModule always runs in hardware slot 3
  m_MotorPwmModule->switchToHardware(3);
  m_MotorPwmModule->setIsActive(true);
}

Configuration::~Configuration()
{
  delete m_SonarModule;
  delete m_LedModule;
  delete m_WhitePixelProcessModule;
  delete m_MotorPwmModule;
}

void Configuration::setModuleConfig(int moduleId, int state, int slot)
{
  if( moduleId < 0 || moduleId >=MODULE_COUNT)
    return;

  if(state == 0) { // deactivate
    for(int i=0;i<SLOT_COUNT;i++){
      if(m_Slots[i] == moduleId){
        m_Slots[i]=-1;
      }
    }
    switch(moduleId) {
      case 0: // Sonar
	m_SonarModule->setIsActive(false);
        break;
      case 1: // LedStop
        m_LedModule->setIsActive(false);
        break;
      case 2: // WhitePixelProcess
        m_WhitePixelProcessModule->setIsActive(false);
        break;
      default:
        break;
    }

  } else if(state == 1){ // switch to hardware
    if (slot < 0 || slot >= SLOT_COUNT){
      return;
    }
    if(m_Slots[slot]!=moduleId && m_Slots[slot]>=0){
      setModuleConfig(m_Slots[slot],0,0);
    }
    for(int i=0;i<SLOT_COUNT;i++) {
      if(i!=slot && m_Slots[i]==moduleId){
        m_Slots[i]=-1;
      }
    }
    switch(moduleId) {
      case 0: // Sonar
        m_SonarModule->switchToHardware(slot);
        m_SonarModule->setIsActive(true);
        m_Slots[slot]=moduleId;
        break;
      case 1: // LedStop
        m_LedModule->switchToHardware(slot);
        m_LedModule->setIsActive(true);
        m_Slots[slot]=moduleId;
        break;
      case 2: // WhitePixelProcess
        m_WhitePixelProcessModule->switchToHardware(slot);
        m_WhitePixelProcessModule->setIsActive(true);
        m_Slots[slot]=moduleId;
        break;
      default:
        break;
    }
  } else if(state == 2) { // switch to software
    switch(moduleId) {
      case 0: // Sonar
        if(m_SonarModule->hasSoftware()) {
          for(int i=0;i<SLOT_COUNT;i++) {
            if(i!=slot && m_Slots[i]==moduleId){
              m_Slots[i]=-1;
            }
          }
          m_SonarModule->switchToSoftware();
          m_SonarModule->setIsActive(true);
        }
        break;
      case 1: // LedStop
        if(m_LedModule->hasSoftware()) {
          for(int i=0;i<SLOT_COUNT;i++) {
            if(i!=slot && m_Slots[i]==moduleId){
              m_Slots[i]=-1;
            }
          }
        
          m_LedModule->switchToSoftware();
          m_LedModule->setIsActive(true);
        }
        break;
      case 2: // WhitePixelProcess
        if(m_WhitePixelProcessModule->hasSoftware()) {
          for(int i=0;i<SLOT_COUNT;i++) {
            if(i!=slot && m_Slots[i]==moduleId){
              m_Slots[i]=-1;
            }
          }
          m_WhitePixelProcessModule->switchToSoftware();
          m_WhitePixelProcessModule->setIsActive(true);
        }
        break;
      default:
        break;
    }
  }
}

