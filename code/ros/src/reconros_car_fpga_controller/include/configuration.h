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

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define MODULE_COUNT 3
#define SLOT_COUNT 3

#include "ros/ros.h"
#include "sonarmodule.h"
#include "ledstopmodule.h"
#include "whitepixelprocessmodule.h"
#include "motorpwmmodule.h"

class Configuration
{
  public:
    explicit Configuration(ros::NodeHandle& node);
    virtual ~Configuration();
    void setModuleConfig(int moduleId, int state, int slot);
  private:
    int m_Slots[SLOT_COUNT];
    SonarModule* m_SonarModule;
    LedStopModule* m_LedModule;
    WhitePixelProcessModule* m_WhitePixelProcessModule;
    MotorPwmModule* m_MotorPwmModule;
};

#endif // CONFIGURATION_H
