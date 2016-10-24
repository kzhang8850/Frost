/*
 Mini Launcher Power Control
 */

#include <math.h>

//Button stuff
int switchValue = 0;
int Switch = 0;

//DC motor stuff
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *Motor = AFMS.getMotor(4);

int MotorSpeed = 0;
int angle = 0;
int Encoder = A1;

//Servo stuff
#include <Servo.h>

Servo latch;

int distance;
int tilt;

//user input for power and angle
String input;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  latch.attach(9);
  latch.write(0);
  
  AFMS.begin();
  Motor->setSpeed(MotorSpeed);
  Motor->run(FORWARD);
  Motor->run(RELEASE);

  pinMode(Switch, INPUT);
  
  resetLauncher();
  distance = 0;
  angle = 0;

  

}

void loop() {
  // put your main code here, to run repeatedly:
  switchValue = digitalRead(Switch);
  if(switchValue == HIGH){
    activateLatch();
  }
  
  if(Serial.available()){
    
    distance = Serial.parseInt();
    armLauncher(distance);
    deactivateLatch();
    delay(1000);
    resetLauncher();
    
    Serial.println("I have fired.");
          
  }
  
  

}
void activateLatch(){
  latch.write(90);
  delay(15);
  MotorSpeed = 0;
  Motor ->setSpeed(MotorSpeed);
  Motor ->run(RELEASE);
}

void armLauncher(int distance){
  int velocity = 10;
  tilt = .5 * asin(9.81 * pow(distance, 2) / velocity);

  angle = analogRead(Encoder);
  while(angle < tilt){
    angle = analogRead(Encoder);
    MotorSpeed = 50;
    Motor ->setSpeed(MotorSpeed);
    Motor ->run(FORWARD);
  }

  
}

void deactivateLatch(){
  latch.write(0);
  delay(15);
  MotorSpeed = 5s;
  Motor ->setSpeed(MotorSpeed);
  Motor ->run(FORWARD);
  
}

void resetLauncher(){
  angle = analogRead(Encoder);
  while(angle > 0){
    angle = analogRead(Encoder);
    MotorSpeed = 15;
    Motor ->setSpeed(MotorSpeed);
    Motor ->run(BACKWARD);
  }
  MotorSpeed = 0;
  Motor ->setSpeed(MotorSpeed);
  Motor ->run(RELEASE);

  
}



