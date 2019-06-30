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
#include <Esplora.h>

#define AUTO_MODE 0
#define MANUAL_MODE 1

#define SERIAL_BAUDRATE 9600
#define SERIAL1_BAUDRATE 9600

#define BUTTON_COUNT 4

const byte buttons[] = {
  SWITCH_RIGHT, 
  SWITCH_LEFT, 
  SWITCH_UP, 
  SWITCH_DOWN 
};

typedef struct {
  char mode;
  char throttle;
  char steering;
  boolean button[BUTTON_COUNT];
} state_t;

state_t state;

void setup() {
  Serial.begin(SERIAL_BAUDRATE);   // Debug information
  Serial1.begin(SERIAL1_BAUDRATE); // XBee
  delay(1000);

  state.mode = AUTO_MODE;
  state.throttle = 0;
  state.steering = 0;
  for (int i = 0; i < BUTTON_COUNT;i++){
    state.button[i] = false;
  }
}

void loop() {
  state_t newstate;

  // read current position of joystick and compute steering and throttle values
  int steering = Esplora.readJoystickX();
  newstate.steering = map(steering,511, -512, 65, 115);

  int throttle = Esplora.readJoystickY();
  newstate.throttle = map(throttle,511, -512,  74, 106);

  // read slider. left is manual mode and right auto mode
  int slider = Esplora.readSlider();
  if (slider < 512){
    newstate.mode = AUTO_MODE;
  }else{
    newstate.mode = MANUAL_MODE;
  }

  // read the state of the buttons
  for(char i = 0;i<BUTTON_COUNT;i++) {
    newstate.button[i] = Esplora.readButton(buttons[i]) == LOW;
  }

  // send state if necessary
  if(state.mode != newstate.mode){
    Serial1.print("M");
    Serial1.print(newstate.mode);
    if(newstate.mode == MANUAL_MODE){
      Serial1.print("T");
      Serial1.print(newstate.throttle);
      Serial1.print("S");
      Serial1.print(newstate.steering);
    }
  } else if(newstate.mode == MANUAL_MODE) { // only send steering and throttle values, if in manual mode
    if(state.throttle != newstate.throttle){
      Serial1.print("T");
      Serial1.print(newstate.throttle);
    }
    if(state.steering != newstate.steering){
      Serial1.print("S");
      Serial1.print(newstate.steering);
    }
  }

  // send button states
  for(char i=0; i<BUTTON_COUNT; i++) {
    if( newstate.button[i]!=state.button[i] && newstate.button[i]) {
      Serial1.print("B");
      Serial1.print((char)i);
    }
  }
  // send debug info
  char val[4];
  Serial.print("M");
  Serial.print((char)('0'+newstate.mode));
  Serial.print("T");
  sprintf(val,"%03d",newstate.throttle);
  Serial.print(val);
  Serial.print("S");
  sprintf(val,"%03d",newstate.steering);
  Serial.print(val);

  for(char i=0; i<BUTTON_COUNT; i++) {
    if( newstate.button[i]!=state.button[i] && newstate.button[i]) {
      Serial.print("B");
      Serial.print((char)('0'+i));
    }
  }
  Serial.println("");
  
  state = newstate;

  delay(100); // wait 100ms
}
