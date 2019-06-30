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

#ifndef RGBWHITEPIXELMODULE_H
#define RGBWHITEPIXELMODULE_H
#include <boost/bind.hpp>
#include <thread>
#include "module.h"
#include "ros/ros.h"

#include <reconros_p2.h>

#include "reconros_car_msgs/ImageWhiteFilter.h"

class RgbWhitePixelModule:public Module
{
  public:
    explicit RgbWhitePixelModule(ros::NodeHandle& node);
    virtual ~RgbWhitePixelModule();

    void setIsActive(bool value);
    void switchToHardware(int slot);
    bool isUsingHardware();
  private:
    ros::Publisher m_WhitePixel_Pub;
    void startProcess();
    void process();
    std::thread m_ProcessThread; 
};

#endif // RGBWHITEPIXELMODULE_H
