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
#include <string>
#include <unistd.h>
#include "ros/ros.h"
#include "reconros_car_msgs/ScenarioState.h"
#include "reconros_car_msgs/ScenarioStateRequest.h"
#include "reconros_car_msgs/CarConfiguration.h"
#include "reconros_car_msgs/RCButton.h"

#include "state.h"

#define MAX_CONFIGURATION 3

void sendState(const ros::Publisher& state_pub, State& state){
  reconros_car_msgs::ScenarioState msg;
  msg.running = state.running();
  msg.paused = state.paused();
  msg.mode = state.mode();
  state_pub.publish(msg);
}

void sendConfiguration(const ros::Publisher& config1_pub, const ros::Publisher& config2_pub, State& state) {
  if(state.configuration() == 0) { // run sonar and white pixel process in software
    reconros_car_msgs::CarConfiguration msg;
    msg.module = 0;
    msg.slot = 0;
    msg.state = 2;
    config1_pub.publish(msg);
  
    msg.module = 1;
    msg.slot = 1;
    msg.state = 0;
    config1_pub.publish(msg);
  
    msg.module = 2;
    msg.slot = 2;
    msg.state = 2;
    config1_pub.publish(msg);
  }else if(state.configuration() == 1){ // run white pixel process in hardware, but sonar in software
    reconros_car_msgs::CarConfiguration msg;
    msg.module = 0;
    msg.slot = 0;
    msg.state = 2;
    config1_pub.publish(msg);
  
    msg.module = 1;
    msg.slot = 1;
    msg.state = 0;
    config1_pub.publish(msg);
  
    msg.module = 2;
    msg.slot = 2;
    msg.state = 1;
    config1_pub.publish(msg);
    
  }else if(state.configuration() == 2){ // run sonar and white pixel process in hardware
    reconros_car_msgs::CarConfiguration msg;
    msg.module = 0;
    msg.slot = 0;
    msg.state = 1;
    config1_pub.publish(msg);
  
    msg.module = 1;
    msg.slot = 1;
    msg.state = 0;
    config1_pub.publish(msg);
  
    msg.module = 2;
    msg.slot = 2;
    msg.state = 1;
    config1_pub.publish(msg);
  }else if(state.configuration() == 3){ // only use led module
    reconros_car_msgs::CarConfiguration msg;
    msg.module = 0;
    msg.slot = 0;
    msg.state = 0;
    config1_pub.publish(msg);
  
    msg.module = 1;
    msg.slot = 1;
    msg.state = 1;
    config1_pub.publish(msg);
  
    msg.module = 2;
    msg.slot = 2;
    msg.state = 0;
    config1_pub.publish(msg);
  } 
}

void stateRequestCallback(const reconros_car_msgs::ScenarioStateRequest::ConstPtr& msg,
                          const ros::Publisher& state_pub,
                          const ros::Publisher& config1_pub,
                          const ros::Publisher& config2_pub,
                          State& state) {
  if(msg->property=="running") {
    if(msg->value=="1"){
      state.setRunning(true);
      state.setPaused(false);
    }else{
      state.setRunning(false);
      sendConfiguration(config1_pub,config2_pub,state);
    }
    sendState(state_pub,state);
  }else if(msg->property=="paused"){
    if(msg->value=="1"){
      state.setPaused(true);
    }else{
      state.setPaused(false);
    }
    /*reconros_car_msgs::ScenarioState msg;
    msg.running = state.running();
    msg.paused = state.paused();
    msg.mode = state.mode();
    state_pub.publish(msg);*/
    sendState(state_pub,state);
  }else if(msg->property=="mode"){
    if(msg->value=="1"){
      state.setMode(1);
    }else{
      state.setMode(0);
    }
    sendState(state_pub,state);   
  } 
}

void buttonCallback(const reconros_car_msgs::RCButton::ConstPtr& msg,
                    const ros::Publisher& state_pub,
                    const ros::Publisher& config1_pub,
                    const ros::Publisher& config2_pub,
                    State& state){
  if (msg->button == 0){
    state.setRunning(false);
    sendState(state_pub,state);
  } else if (msg->button == 1) {
    state.setRunning(true);
    sendConfiguration(config1_pub,config2_pub,state);
    sendState(state_pub,state);
  } else if (msg->button == 2) {
    bool unpause = false;
    if (state.configuration() >0) {
      state.setConfiguration(state.configuration()-1);
      if (state.paused()){
        unpause=true;
      } 
    }else{
      state.setConfiguration(0);
    }
    sendConfiguration(config1_pub,config2_pub, state); 
    if(unpause){
      usleep(100);
      state.setPaused(false);
      sendState(state_pub,state);
    }
  } else if (msg->button == 3) {
    if (state.configuration() < MAX_CONFIGURATION) {
      state.setConfiguration(state.configuration()+1);
    }else{
      state.setConfiguration(MAX_CONFIGURATION);
    }
    sendConfiguration(config1_pub,config2_pub, state); 
  }
}


int main(int argc, char **argv) {
  State state;
  ros::init(argc, argv, "scenariomanager");
  ros::NodeHandle n;
  ros::Publisher state_pub = n.advertise<reconros_car_msgs::ScenarioState>("state",10);
  ros::Publisher config_p1_pub = n.advertise<reconros_car_msgs::CarConfiguration>("configuration1",10);
  ros::Publisher config_p2_pub = n.advertise<reconros_car_msgs::CarConfiguration>("configuration2",10);


  ros::Subscriber state_sub = n.subscribe<reconros_car_msgs::ScenarioStateRequest>("staterequest", 10,
                                                                                   boost::bind(stateRequestCallback,
                                                                                   _1,
                                                                                   boost::ref(state_pub),
                                                                                   boost::ref(config_p1_pub),
                                                                                   boost::ref(config_p2_pub),
                                                                                   boost::ref(state)));
  ros::Subscriber button_sub = n.subscribe<reconros_car_msgs::RCButton>("buttons", 10,
                                                                        boost::bind(buttonCallback,
                                                                        _1,
                                                                        boost::ref(state_pub),
                                                                        boost::ref(config_p1_pub),
                                                                        boost::ref(config_p2_pub),
                                                                        boost::ref(state)));

  ros::spin();
  return 0;
}
