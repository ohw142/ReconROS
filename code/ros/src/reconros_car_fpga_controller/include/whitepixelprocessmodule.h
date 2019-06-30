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

#ifndef WHITEPIXELPROCESSMODULE_H
#define WHITEPIXELPROCESSMODULE_H
#include <boost/bind.hpp>
#include "module.h"
#include "ros/ros.h"

#include <reconros_p1.h>

#include "reconros_car_msgs/CarSteering.h"
#include "reconros_car_msgs/ScenarioState.h"
#include "reconros_car_msgs/ImageWhiteFilter.h"

class WhitePixelProcessModule: public Module
{
  public:
    explicit WhitePixelProcessModule(ros::NodeHandle& node);
    virtual ~WhitePixelProcessModule();

    void switchToSoftware();
    void switchToHardware(int slot);
    bool isUsingHardware();
    void processData(std::vector<uint32_t> bins);
    void processState(bool running);
  private:
    ros::Subscriber m_White_Pixels_Sub;
    ros::Subscriber m_State_Sub;
    ros::Publisher m_Steering_Pub;
    bool m_UseHardware;
    bool m_Running;
};

#endif // WHITEPIXELPROCESSMODULE_H
