/* LidarLite v2 laser mounted on 3d printed chassis

  Copyright (c) 2015 Alexander Grau 
  Private-use only! (you need to ask for a commercial-use) 
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  Private-use only! (you need to ask for a commercial-use)
 */

// 
// serial output:
// [0] angle (degree) high byte
// [1] angle (degree) low byte
// [3] distance (cm) high byte
// [4] distance (cm) low byte
// ...repeats
//
// ...until zero pos indication: 
// 0xCC, 0xDD, 0xEE, 0xFF


// Arduino Nano wiring: 
// A5  - Lidar SCL 
// A4  - Lidar SDA
// D4  - Lidar Mode (Add Pull-Down!)  NOT USED!
// D3  - disc encoder pos 
// D5  - disc encoder LED (Active Pull-Down )  NOT USED!


// diagnostic mode: open Arduino IDE serial console (CLTR+SHIFT+M), and send 'd' to the Lidar
// diagnostic output should be like this:
// time=239  freq=3  angleMin=0  angleMax=363
// time=240  freq=3  angleMin=1  angleMax=363
// time=241  freq=3  angleMin=1  angleMax=364


#include <Arduino.h>
#include <Wire.h>
#include "LIDARLite.h"
#include "TimerOne.h"
#include <ros.h>
#include <std_msgs/UInt8.h>

int pinLidarMode   = 4;
int pinEncoderPos  = 3;
int pinEncoderLED  = 5;
int pinLED  = 13;


#define MODE_NORMAL  0
#define MODE_DEBUG   1


char mode = MODE_NORMAL;
LIDARLite myLidarLite;
volatile int angle = 0;
volatile float ratio = 0;
volatile bool zeroPos = false;
volatile int tickLowDuration = 0;
volatile int tickHighDuration = 0;
volatile int tickAngle = 0;
volatile unsigned long tickLastTime = 0;
int measurements = 0;
unsigned long nextSerialTime = 0;
unsigned long nextInfoTime = 0;
unsigned long rotationCounter = 0;
int angleMax = 0;
int angleMin = 30000;


ros::NodeHandle nh;
std_msgs::UInt8 hex_msg;
ros::Publisher chatter("chatter", &hex_msg);


void timer(){
  angle++; // set to next angle step
}

void encoderTick(){
  volatile byte state = digitalRead(pinEncoderPos);  
  volatile unsigned long ms = millis();          
  digitalWrite(pinLED, state);
  volatile int tickDuration = ms-tickLastTime;   // compute transition time        
  if (tickDuration == 0) tickDuration = 1;
  tickAngle+=12;
  if (state == LOW){ 
    // high->low transition
    tickHighDuration = tickDuration;   // remember high time         
    ratio = ((float)tickLowDuration)/((float)tickHighDuration);    
    if (ratio < 0.66){    
      // zero pos      
      zeroPos = true;
      tickAngle = 0;     // reset angle to zero                     
      angle = tickAngle;
    } else {
      angle = tickAngle;
      Timer1.setPeriod(tickDuration*1000/12); // set degree timer based on transition time                         
    }
  } else { 
    // low->high transition
    tickLowDuration = tickDuration;   // remember low time                
  }     
  tickLastTime = ms; // remember transition time  
}

void setup() {
  
  //myLidarLite.begin();
  pinMode(pinLidarMode, INPUT);
  pinMode(pinEncoderPos, INPUT);
  pinMode(pinEncoderLED, INPUT);
  pinMode(pinLED, OUTPUT);
  
  // begin(int configuration, bool fasti2c, bool showErrorReporting, char LidarLiteI2cAddress)
  myLidarLite.begin(1, true);
  //myLidarLite.configure(0);
  Serial.begin(115200);

  Timer1.initialize(1000000); // 1 second
  Timer1.attachInterrupt(timer);  
  
  nh.advertise(chatter);
  nh.initNode();
  //Serial.println("Ready");
  attachInterrupt(digitalPinToInterrupt(pinEncoderPos), encoderTick, CHANGE);
}


void loop() {
  if (millis() >= nextSerialTime){
    nextSerialTime = millis() + 500;
    if (Serial.available()){
      char ch = Serial.read();
      if (ch == 'd') mode = MODE_DEBUG;
      if (ch == 'n') mode = MODE_NORMAL;
    }  
  }
    
  int d = 0;
  //d = myLidarLite.distance();
  while (digitalRead(pinLidarMode) == HIGH);
  //d = myLidarLite.distance(false);
  //int s = myLidarLite.signalStrength();
  int v = angle;
  if (v <2){
    // take 1 reading with preamp stabilization and reference pulse (these default to true)
    d = myLidarLite.distance();
  } else {
    // take reading without preamp stabilization and reference pulse (these read about 0.5-0.75ms faster than with)
    d = myLidarLite.distance(false);
  }
  if (zeroPos){
    // zero point     
    zeroPos = false;    
    if (mode == MODE_NORMAL){
      /*// fill-up to 360 measurements
      while (measurements < 360){      
        Serial.write(0);  
        Serial.write(0);    
        Serial.write(0);  
        Serial.write(0); 
        Serial.flush();         
        measurements++;
      }*/      
      // send zero sync
      hex_msg.data = 0xCC;
      chatter.publish(&hex_msg);
      hex_msg.data = 0xDD;
      chatter.publish(&hex_msg);
      hex_msg.data = 0xEE;
      chatter.publish(&hex_msg);
      hex_msg.data = 0xFF;
      chatter.publish(&hex_msg);ls
    } else {
      //Serial.println(rotationCounter);
      //Serial.flush();
    }
    measurements = 0;
    rotationCounter++;
  }

  if (mode == MODE_NORMAL){
    hex_msg.data = v >> 8;
    chatter.publish(&hex_msg);
    hex_msg.data = v & 0xFF;
    chatter.publish(&hex_msg);
    hex_msg.data = d >> 8;
    chatter.publish(&hex_msg);
    hex_msg.data = d & 0xFF;
    chatter.publish(&hex_msg);
  } 
  else {
  
    if (angle > angleMax) angleMax = angle;
    if (angle < angleMin) angleMin = angle;
    if (millis() >= nextInfoTime){
      nextInfoTime = millis() + 1000;
      Serial.print("time=");
      Serial.print(millis()/1000);
      Serial.print("  freq=");
      Serial.print(rotationCounter);
      Serial.print("  angleMin=");
      Serial.print(angleMin);
      Serial.print("  angleMax=");
      Serial.print(angleMax);
      Serial.println();

      angleMax=0;
      angleMin=30000;
      rotationCounter=0;
      /*Serial.print("idx=");
      Serial.print(measurements);
      Serial.print("  angle=");
      Serial.print(v);
      Serial.print("  distance(cm)=");
      Serial.println(d);  */
      //Serial.print("  strength=");
      //Serial.println(s);  */       
    }    
  }
  measurements++;
  nh.spinOnce();
} 
