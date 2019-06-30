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

#ifndef MOTORPWMMODULE_H
#define MOTORPWMMODULE_H
#include <boost/bind.hpp>
#include "module.h"
#include "ros/ros.h"

#include <reconros_p1.h>

#include "reconros_car_msgs/CarThrottle.h"
#include "reconros_car_msgs/CarSteering.h"

class MotorPwmModule:public Module
{
  public:
    explicit MotorPwmModule(ros::NodeHandle& node);
    virtual ~MotorPwmModule();

    void switchToHardware(int slot);

    void processSteering(int8_t steeringValue);
    void processThrottle(int8_t throttleValue);
  private:
    ros::Subscriber m_Steering_Sub;
    ros::Subscriber m_Throttle_Sub;
};

#endif // MOTORPWMMODULE_H
