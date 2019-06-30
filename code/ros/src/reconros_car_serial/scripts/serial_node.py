#!/usr/bin/python

# ReconROS 
# Copyright (C) 2019  Andreas Krakau, Felix Paul Jentzsch
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import rospy
import serial 
import time
from reconros_car_msgs.msg import CarThrottle, CarSteering, CarSonar, CarLEDArray, RCButton, ScenarioStateRequest

def serial_connect():
  return serial.Serial(port='/dev/ttyACM0',
                  baudrate=115200,
                  parity=serial.PARITY_NONE,
                  stopbits=serial.STOPBITS_ONE,
                  bytesize=serial.EIGHTBITS,
                  timeout=1
  )

def serial_node():
  serial = serial_connect()
  
  rospy.init_node('serial_node', anonymous=True)

  sonar_pub = rospy.Publisher('sonar', CarSonar, queue_size=10)
  ledarray_pub = rospy.Publisher('ledarray', CarLEDArray, queue_size=10)
  buttons_pub = rospy.Publisher('buttons', RCButton, queue_size=10)
  mode_pub = rospy.Publisher('staterequest', ScenarioStateRequest, queue_size=10)
  steering_pub = rospy.Publisher('steeringmanualrequest', CarSteering, queue_size = 10)
  throttle_pub = rospy.Publisher('throttlemanualrequest', CarThrottle, queue_size = 10)
 
  msg_type = ''
  
  while not rospy.is_shutdown():
    # Read Serial if available
    while serial.in_waiting:
      if msg_type == '':
        msg_type = serial.read()
      else:
        if msg_type == 'E':
          value = ord(serial.read())
          msg = CarSonar()
          msg.sonar = value
          sonar_pub.publish(msg)
        elif msg_type == 'I':
          msg = CarLEDArray()
          msg.leds = ord(serial.read())
          ledarray_pub.publish(msg)
        elif msg_type == 'B':
          msg = RCButton()
          msg.button = ord(serial.read())
          buttons_pub.publish(msg)
        elif msg_type == 'R':
          msg = ScenarioStateRequest()
          msg.property = "mode"
          msg.value = "%d"%ord(serial.read())
          mode_pub.publish(msg)
        elif msg_type == 'S':
          msg = CarSteering()
          msg.steering = ord(serial.read())-90
          steering_pub.publish(msg)
        elif msg_type == 'T':
          msg = CarThrottle()
          msg.throttle = ord(serial.read())-90
          throttle_pub.publish(msg)
        msg_type = ''
    time.sleep(0.01)
     
if __name__ == "__main__":
  serial_node()
