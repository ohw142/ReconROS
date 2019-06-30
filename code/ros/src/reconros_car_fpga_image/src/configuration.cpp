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
  m_RgbWhitePixelModule(new RgbWhitePixelModule(node))
{
  // RgbWhitePixelModule is the default image module
  m_RgbWhitePixelModule->switchToHardware(0);
  m_RgbWhitePixelModule->setIsActive(true);
}

Configuration::~Configuration()
{
  delete m_RgbWhitePixelModule;
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
      case 0: // RgbWhitePixels
        m_RgbWhitePixelModule->setIsActive(false);
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
      case 0: // RgbWhitePixel
        m_RgbWhitePixelModule->switchToHardware(slot);
        m_RgbWhitePixelModule->setIsActive(true);
        m_Slots[slot]=moduleId;
        break;
      default:
        break;
    }
  } else if(state == 2) { // switch to software
    switch(moduleId) {
      case 0: // RgbWhitePixel
        if(m_RgbWhitePixelModule->hasSoftware()) {
          for(int i=0;i<SLOT_COUNT;i++) {
            if(i!=slot && m_Slots[i]==moduleId){
              m_Slots[i]=-1;
            }
          }
          m_RgbWhitePixelModule->switchToSoftware();
          m_RgbWhitePixelModule->setIsActive(true);
        }
        break;
      default:
        break;
    }
  }
}
