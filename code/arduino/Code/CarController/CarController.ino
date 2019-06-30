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
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <Wire.h>

#define uchar_t unsigned char

#define AUTO_MODE 0
#define MANUAL_MODE 1

// Pins
#define SONAR_TRIGGER_PIN 8
#define SONAR_ECHO_PIN 9

#define SOFTWARE_SERIAL_RX_PIN 10
#define SOFTWARE_SERIAL_TX_PIN 11
#define SOFTWARE_SERIAL_BAUDRATE 9600
#define HARDWARE_SERIAL_BAUDRATE 115200

#define SONAR_MAX_DISTANCE 200
#define IR_THRESHOLD 150


SoftwareSerial RC_Serial(SOFTWARE_SERIAL_RX_PIN,SOFTWARE_SERIAL_TX_PIN);

NewPing sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, SONAR_MAX_DISTANCE);

QueueHandle_t sonarQueue;    // Transmit sonar values from 'Sonar' task to 'Serial' task
QueueHandle_t ledArrayQueue; // Transmit led array state from 'LED' task to 'Serial' task


void TaskSonar( void *pvParameters );
void TaskLedArray( void *pvParameters );
void TaskSerial( void *pvParameters );

void setup() {
  Serial.begin(HARDWARE_SERIAL_BAUDRATE);
  while (!Serial) {}
  RC_Serial.begin(SOFTWARE_SERIAL_BAUDRATE);
  
  Wire.begin();
  delay(1000);

  sonarQueue = xQueueCreate(5, sizeof(uchar_t));
  ledArrayQueue = xQueueCreate(1, sizeof(uchar_t));

  xTaskCreate(
    TaskSonar
    ,  (const portCHAR *)"Sonar"
    ,  128  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL );

  xTaskCreate(
    TaskLedArray
    ,  (const portCHAR *) "LED"
    ,  128  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL );

  xTaskCreate(
    TaskSerial
    ,  (const portCHAR *) "Serial"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );
}

void loop() {
  // Nothing to do here
}

/**
 * Reads the current value of the sonar sensor
 */
void TaskSonar(void *pvParameters)
{
  (void) pvParameters;
  const TickType_t period = 50; // Every 50ms
  TickType_t last;
  uchar_t distance=0;
  last = xTaskGetTickCount();
  
  for (;;)
  {
    distance = (uchar_t)sonar.ping_cm();
    if(distance>0){
      xQueueSend(sonarQueue,&distance,0);
    }
    vTaskDelayUntil(&last, ( period / portTICK_PERIOD_MS ));
  }
}

/**
 * Reads the current state of the infrared leds 
 */
void TaskLedArray(void *pvParameters)
{
  (void) pvParameters;
  const TickType_t period = 200; // Every 200ms
  TickType_t last;
  uchar_t i = 0;
  uchar_t data[16]; 
  uchar_t ledArray=0;

  last = xTaskGetTickCount();
  
  for (;;)
  {
    Wire.requestFrom(9, 16);
    i=0;
    while(Wire.available()&&i<16)
    {
      data[i++]=Wire.read();   
    }
    
    ledArray=0;
    for(i=0;i<8;i++) {
      if (data[2*i]>IR_THRESHOLD){
        ledArray|=1<<(7-i);
      }
    }
    
    xQueueSend(ledArrayQueue,&ledArray,0);
    vTaskDelayUntil(&last, ( period / portTICK_PERIOD_MS ));
  }
}

void TaskSerial(void *pvParameters)
{
  (void) pvParameters;
  const TickType_t period = 50; // Every 50ms
  TickType_t last;
  char serial_buffer='\0';
  char rc_serial_buffer='\0';
  char serial_message='\0';

  char mode = AUTO_MODE;
  uchar_t throttle = 0;
  uchar_t steering = 0;
  uchar_t sonarDistance = 0;
  uchar_t ledArray = 0;
  
  last = xTaskGetTickCount();
  
  for (;;)
  {
    /* Receive messages from remote control:
     *  Messages are two byte long. First byte indicates the type of the message.
     *  The second byte contains the message content.
     */
    while(RC_Serial.available()) {
      serial_message=RC_Serial.read();

      if(rc_serial_buffer=='M'){ // mode changed
        if(serial_message == MANUAL_MODE){
          mode=MANUAL_MODE;

        } else {
          mode=AUTO_MODE;
        }
        Serial.print("R");
        Serial.print(mode);
        rc_serial_buffer='\0';
      }else if(rc_serial_buffer=='T'){ // throttle changed
        if(mode == MANUAL_MODE) {
          throttle = (uchar_t)(serial_message); 
          Serial.print("T");
          Serial.print((char)throttle);
        }
        rc_serial_buffer='\0';
      }else if(rc_serial_buffer=='S'){ // steering changed
        if(mode == MANUAL_MODE) {
          steering = (uchar_t)(serial_message); 
          Serial.print("S");
          Serial.print((char)steering);
        }
        rc_serial_buffer='\0';
      }else if(rc_serial_buffer=='B'){ // button pressed
        Serial.print("B");
        Serial.print(serial_message);
        rc_serial_buffer='\0';
      }else{ // 
        rc_serial_buffer=serial_message;
      }
    }
    // Send the last sonar value
    if (xQueueReceive(sonarQueue,&sonarDistance,0) == pdTRUE){
      Serial.print("E");
      Serial.print((char)sonarDistance);
    }
    // Send the last led array state
    if (xQueueReceive(ledArrayQueue,&ledArray,0) == pdTRUE){
      Serial.print("I");
      Serial.print((char)ledArray);
    }
    vTaskDelayUntil(&last, ( period / portTICK_PERIOD_MS ));
  }
}
