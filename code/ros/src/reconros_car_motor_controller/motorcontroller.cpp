#include <boost/bind.hpp>
#include "ros/ros.h"

#include "reconros_car_msgs/CarThrottle.h"
#include "reconros_car_msgs/CarSteering.h"
#include "reconros_car_msgs/ScenarioState.h"

#include "state.h"

void steeringCallback(const reconros_car_msgs::CarSteering::ConstPtr& msg, const ros::Publisher& pub, State& state) {
  int steering = msg->steering;
  if(steering<-25)
    steering = -25;
  if(steering>25)
    steering = 25;
  state.setSteering(steering);
  reconros_car_msgs::CarSteering msg;
  msg.steering = steering;
  pub.publish(msg);
}

void throttleCallback(const reconros_car_msgs::CarThrottle::ConstPtr& msg, State& state) {
  
}

void stateCallback(const reconros_car_msgs::ScenarioState::ConstPtr& msg, State& state) {
  state.setRunning(msg->running);
  state.setPaused(msg->paused);
}

int main(int argc, char **argv) {
  ros::init(argc,argv,"motor_controller");
  ros::NodeHandle n;

  State state;

  ros::Publisher throttle_pub = n.advertise<reconros_car_msgs::CarThrottle>("throttle",10);
  ros::Publisher steering_pub = n.advertise<reconros_car_msgs::CarSteering>("steering",10)

  ros::Subscriber state_sub = n.subscribe<reconros_car_msgs::ScenarioState>("state", 10, boost::bind(stateCallback, _1, boost::ref(state)));
  ros::Subscriber steering_sub = n.subscribe<reconros_car_msgs::CarSteering>("steeringrequest", 10, boost::bind(steeringCallback, _1, boost::ref(steering_pub), boost::ref(state)));
  ros::Subscriber throttle_sub = n.subscribe<reconros_car_msgs::CarThrottle>("throttlerequest", 10, boost::bind(throttleCallback, _1, boost::ref(state)));

  ros::Rate rate(10);

  while(ros::ok()) {
    if (state.running() && !state.paused()) {
      reconros_car_msgs::CarThrottle msg;
      msg.throttle = 10;
      throttle_pub.publish(msg);
    } else {
      reconros_car_msgs::CarThrottle msg;
      msg.throttle = 0;
      throttle_pub.publish(msg);
    }

    ros::spin_once();
    rate.sleep();
  }

  return 0;
}


