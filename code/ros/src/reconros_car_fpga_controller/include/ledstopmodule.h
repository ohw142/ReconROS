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

#ifndef LEDSTOPMODULE_H
#define LEDSTOPMODULE_H

#include "module.h"
#include "ros/ros.h"

#include <reconros_p1.h>

#include "reconros_car_msgs/ScenarioState.h"
#include "reconros_car_msgs/ScenarioStateRequest.h"
#include "reconros_car_msgs/CarLEDArray.h"
#include "reconros_car_msgs/CarSteering.h"

class LedStopModule:public Module
{
  public:
    explicit LedStopModule(ros::NodeHandle& node);
    virtual ~LedStopModule();

    void switchToSoftware();
    void switchToHardware(int slot);
    bool isUsingHardware();
    void processLed(uint32_t ledValue);
    void processState(bool running, bool paused);
  private:
    ros::Subscriber m_Led_Sub;
    ros::Subscriber m_State_Sub;
    ros::Publisher m_State_Pub;
    ros::Publisher m_Steering_Pub;
    bool m_UseHardware;
    bool m_Running;
    bool m_Paused;
};





#endif // LEDSTOPMODULE_H
