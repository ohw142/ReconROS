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

#include <boost/bind.hpp>
#include "ros/ros.h"
#include "state.h"

#include "reconros_car_msgs/CarSteering.h"
#include "reconros_car_msgs/CarThrottle.h"
#include "reconros_car_msgs/ScenarioState.h"

void stateCallback(const reconros_car_msgs::ScenarioState::ConstPtr& msg,
                   const ros::Publisher& steering_pub,
                   const ros::Publisher& throttle_pub,
                   State& state)
{
  state.setRunning(msg->running);
  state.setPaused(msg->paused);
  if(state.mode()!=msg->mode){
    state.setMode(msg->mode);
    reconros_car_msgs::CarSteering steering_msg;
    steering_msg.steering = state.steering();
    steering_pub.publish(steering_msg);
    reconros_car_msgs::CarThrottle throttle_msg;
    throttle_msg.throttle = state.throttle();
    throttle_pub.publish(throttle_msg);
  }
}

void steeringCallback(const reconros_car_msgs::CarSteering::ConstPtr& msg,
                      const ros::Publisher& steering_pub,
                      State& state)
{
  if(state.mode()==0) {
    if(state.steering()!=msg->steering){
      state.setSteering(msg->steering);
      reconros_car_msgs::CarSteering steering_msg;
      steering_msg.steering = msg->steering;
      steering_pub.publish(steering_msg); 
    }
  }
}

void steeringManualCallback(const reconros_car_msgs::CarSteering::ConstPtr& msg,
                            const ros::Publisher& steering_pub,
                            State& state)
{
  if(state.mode()==1) {
    if(state.steering()!=msg->steering){
      state.setSteering(msg->steering);
      reconros_car_msgs::CarSteering steering_msg;
      steering_msg.steering = msg->steering;
      steering_pub.publish(steering_msg); 
    }
  }
}


void throttleCallback(const reconros_car_msgs::CarThrottle::ConstPtr& msg,
                            const ros::Publisher& throttle_pub,
                            State& state)
{
  if(state.mode()==0) {
    state.setThrottle(msg->throttle);
    reconros_car_msgs::CarThrottle throttle_msg;
    throttle_msg.throttle = msg->throttle;
    throttle_pub.publish(throttle_msg); 
  }
}


void throttleManualCallback(const reconros_car_msgs::CarThrottle::ConstPtr& msg,
                            const ros::Publisher& throttle_pub,
                            State& state)
{
  if(state.mode()==1) {
    state.setThrottle(msg->throttle);
    reconros_car_msgs::CarThrottle throttle_msg;
    throttle_msg.throttle = msg->throttle;
    throttle_pub.publish(throttle_msg); 
  }

}



int main(int argc, char **argv)
{
  ros::init(argc,argv,"motor_controller");
  ros::NodeHandle n;

  State state;

  ros::Publisher steering_pub = n.advertise<reconros_car_msgs::CarSteering>("steering",10);
  ros::Publisher throttle_pub = n.advertise<reconros_car_msgs::CarThrottle>("throttle",10);
  
  ros::Subscriber state_sub = n.subscribe<reconros_car_msgs::ScenarioState>("state", 10,
                                                                            boost::bind(stateCallback,
                                                                                        _1,
                                                                                        boost::ref(steering_pub),
                                                                                        boost::ref(throttle_pub),
                                                                                        boost::ref(state)));
  ros::Subscriber steeringRequest_sub = n.subscribe<reconros_car_msgs::CarSteering>("steeringrequest", 10,
                                                                                    boost::bind(steeringCallback,
                                                                                        _1,
                                                                                        boost::ref(steering_pub),
                                                                                        boost::ref(state)));
  
  ros::Subscriber steeringManualRequest_sub = n.subscribe<reconros_car_msgs::CarSteering>("steeringmanualrequest", 10,
                                                                                          boost::bind(steeringManualCallback,
                                                                                                      _1,
                                                                                                      boost::ref(steering_pub),
                                                                                                      boost::ref(state)));
  
  ros::Subscriber throttleRequest_sub = n.subscribe<reconros_car_msgs::CarThrottle>("throttlerequest", 10,
                                                                                    boost::bind(throttleCallback,
                                                                                                _1,
                                                                                                boost::ref(throttle_pub),
                                                                                                boost::ref(state)));

  ros::Subscriber throttleManualRequest_sub = n.subscribe<reconros_car_msgs::CarThrottle>("throttlemanualrequest", 10,
                                                                                          boost::bind(throttleManualCallback,
                                                                                                      _1,
                                                                                                      boost::ref(throttle_pub),
                                                                                                      boost::ref(state)));

  ros::Rate rate(10);
  int8_t oldThrottle = 0;
  while(ros::ok()) {
    if (state.mode()!=1) {
      if(state.running() && !state.paused()) {
        state.setThrottle(10);
      } else {
        state.setThrottle(0);
      }
      if(state.throttle()!=oldThrottle){
        oldThrottle = state.throttle();
        reconros_car_msgs::CarThrottle msg;
        msg.throttle = state.throttle();
        throttle_pub.publish(msg);
      }
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
