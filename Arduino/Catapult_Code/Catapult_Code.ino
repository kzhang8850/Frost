/*
 Final for big launcher, does Pan and Power calculations
 Uses PID for motor control of Pan, and has user input
 */
#include <math.h>
#include <Servo.h>
//DC Motor Initialization
#include <Wire.h>
#include <Adafruit_MotorShield.h>
//#include "utility/Adafruit_PWMServoDriver.h"



#define limitSwitchPin 2
//Servo
Servo latch1; //70 locked 110 unlocked
Servo latch2; //100 locked 55 unlocked

//MOTORS
Adafruit_MotorShield AFMS(0x60);// = Adafruit_MotorShield();
Adafruit_MotorShield AFMS2(0x61);// = Adafruit_MotorShield();
//Pan Motor

//Winch Motors
Adafruit_DCMotor *winch1 = AFMS.getMotor(1);
Adafruit_DCMotor *winch2 = AFMS.getMotor(4);
Adafruit_DCMotor *winch3 = AFMS2.getMotor(1);
Adafruit_DCMotor *winch4 = AFMS2.getMotor(3);
Adafruit_DCMotor *panMotor = AFMS2.getMotor(4);

//PAN VARIABLES
int panPot = A0;
int panTopSpeed = 80;
float panEncoderValue;    //IN DEGREES
float panOffset = 267; //IN VOLTAGE
float panError = 0;  //IN DEGREES
float panPGain = 5;
float panDGain = 25;
float panIGain = .005;
float panSumError = 0;
float panDerivative = 0;
float panProportional = 0;
float panIntegral = 0;
int panThreshold = 1;
int panDt = 50;
int panTarget = 0;
int panMotorSpeed = 0;
int prevTime = 0;
float truePanSpeed = 0;

//used in String parsing input
int separator;
String string1;
String string2;
String ID;
String stringvalue;
int value;
int len;
int Distance;

//ARM VARIABLES
int armPot = A1;
float armEncoderValue;
float armOffset = 201; // IN VOLTAGE
; //IN DEGREES
bool reset = false;
bool fire = false;
bool arm = false;
int armTarget = 0;
int switchPressed = 0;

String input;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("starting");
  input = "default";

  //sets up motors
  AFMS.begin();
  AFMS2.begin();
  
  latch1.attach(9);
  latch2.attach(10);
  fireLauncher();
  panMotor->run(RELEASE);

  while(!reset){
    getSensorData();
    resetLauncher();
  }
  Serial.println("reset");

}


//main loop
void loop() {
  Serial.setTimeout(50);
  getSensorData();
  if (Serial.available()) {
    getInput();
  }
  pan();
  if(reset && arm){
    armLauncher();
  }
  if(fire && !arm){
    fireLauncher();
    panMotor->run(RELEASE);
    delay(500);
    reset = false;
    fire = false;
  }
  if(!reset){
    panTarget = 0;
    resetLauncher();
  }
}


//reads current values of encoders for control
void getSensorData() {
  //DO NOT COMMENT OUT SWITCH PRESSED
  switchPressed = digitalRead(limitSwitchPin);
  //clockwise values are positive
  panEncoderValue = -voltageToDegrees(analogRead(panPot) - panOffset);
  //Serial.println(analogRead(panPot));
  armEncoderValue = voltageToDegrees(analogRead(armPot) - armOffset);
  //Serial.println(armEncoderValue);
}


//conversion function
float voltageToDegrees(float voltage) {
  // turns raw pot data into degrees
  return voltage/4;
}



//Pan Motor moves to set angle point using PID control
void pan() {
  //Turns target and current encoder value to power output
  if (panTarget > 60) {
    panTarget = 60;
    Serial.println("pan limit reached");
  }
  if (panTarget < -60) {
    panTarget = 60;
    Serial.println("pan limit reached");
  }
  if (millis() - prevTime > panDt) {
    int prevPanEncoderValue = panEncoderValue;
    getSensorData();
    panDerivative = panDGain * (panEncoderValue - prevPanEncoderValue);
    prevTime = millis();
  }
  panError = float(panTarget) - panEncoderValue;
  //Serial.println(panError);
  panSumError += panError;
  //Proportional Term of PID

  panIntegral = panIGain * panSumError;
  panProportional = panPGain *  panError;
  //Calculate new motor speed
  panMotorSpeed = panProportional + panDerivative + panIntegral;

  if (panMotorSpeed > panTopSpeed) {
    panMotorSpeed = panTopSpeed;
  }
  if (panMotorSpeed < -panTopSpeed) {
    panMotorSpeed = -panTopSpeed;
  }
  if(abs(panError) < panThreshold){
    panSumError = 0;
    panMotorSpeed = 0;
  }
  //Serial.println(panError);
  turnpanMotor(panMotorSpeed);
}


//Function for turning the Pan Motor
void turnpanMotor(int motorSpeed) {
  //Rotates the motors
  if(truePanSpeed > motorSpeed){
    truePanSpeed -= .5;
  }else if(truePanSpeed < motorSpeed){
    truePanSpeed += .5;
  }
  if (int(truePanSpeed) > 0) {
    panMotor->setSpeed(int(truePanSpeed) );
    panMotor->run(BACKWARD);
  } else if (truePanSpeed  < 0) {
    panMotor->setSpeed(abs(int(truePanSpeed)));
    panMotor->run(FORWARD);
  } else {
    panMotor ->run(RELEASE);
  }
}


//Function for turning Arm Motors
void turnArmMotors(int motorSpeed) {
  if (motorSpeed > 0) {
    winch1->setSpeed(motorSpeed);
    winch2->setSpeed(motorSpeed);
    winch3->setSpeed(motorSpeed);
    winch4->setSpeed(motorSpeed);
    winch1->run(BACKWARD);
    winch2->run(FORWARD);
    winch3->run(BACKWARD);
    winch4->run(FORWARD);
  } else if (motorSpeed < 0) {
    winch1->setSpeed(abs(motorSpeed));
    winch2->setSpeed(abs(motorSpeed));
    winch3->setSpeed(abs(motorSpeed));
    winch4->setSpeed(abs(motorSpeed));
    winch1->run(FORWARD);
    winch2->run(BACKWARD);
    winch3->run(FORWARD);
    winch4->run(BACKWARD);
  } else {
    winch1->run(RELEASE);
    winch2->run(RELEASE);
    winch3->run(RELEASE);
    winch4->run(RELEASE);
  }
}



//Manual input for testing and human control
void getInput() {
    //INPUT IN FORMAT a = 20, power = 30
    input = Serial.readString();
    
    separator = input.indexOf(".");
    if(separator != -1){
      fire = true;
      Serial.println("FIRING!");
    }
    if(input == "reset"){
      fire = true;
      Serial.println("Resetting launcher");
    }
    len = input.length()+1;
    separator = input.indexOf(",");
    if (separator != -1){
      string1 = input.substring(0, separator);
      string2 = input.substring(separator+1, len);
      string1.trim();
      string2.trim();
      
      separator = string1.indexOf("=");
      len = string1.length()+1;

      ID = string1.substring(0, separator);
      stringvalue = string1.substring(separator+1,len);
      stringvalue.trim();
      value = stringvalue.toInt();
      ID.trim();
      if(ID == "a"){
        panTarget = value;
        Serial.print("Angle set to: ");
        Serial.println(panTarget);
      }
      separator = string2.indexOf("=");
      len = string2.length()+1;
      ID = string2.substring(0, separator);
      stringvalue = string2.substring(separator+1,len);
      stringvalue.trim();
      value = stringvalue.toInt();
      ID.trim();
      if(ID == "power"){
        armTarget = value;
        if(armTarget > 0){
          arm =  true;
          Serial.print("Power set to: ");
          Serial.println(armTarget);
        }else{
          Serial.println("Negative Power Given, try agan");
        }
      }
      }else{
        separator = input.indexOf("=");
        len = input.length()+1;
        ID = input.substring(0, separator);
        stringvalue = input.substring(separator+1,len);
        stringvalue.trim();
        value = stringvalue.toInt();
        ID.trim();
        if(ID == "a"){
          panTarget = value;
          Serial.print("Angle set to: ");
          Serial.println(panTarget);
        }
      }
      Serial.flush();

      
       

}


//Arm motors move back to set distance using essentially bang bang control
void armLauncher() {
  if(armTarget > 60){
    armTarget = 60;
  }
  if(armEncoderValue < armTarget && switchPressed == 0){
    turnArmMotors(150);
  }
  else{
    fire = true;
    arm = false;
    turnArmMotors(10);
    if(switchPressed == 1){
      Serial.println("Limit Reached");
    }
  }
}



//Re-arms the launcher using essentially bang bang control
void resetLauncher() {
  if (armEncoderValue > 0){
    turnArmMotors(-200);
  }else{
    delay(70);
    turnArmMotors(1);
    delay(10);
    turnArmMotors(0);
    delay(200);
    reset = true;
    activateLatch();
    delay(100);
  }
}


//Arm Functions
void activateLatch() {
  //latch2.detach();
  //latch1.detach();
  latch2.write(95);
  latch1.write(60);
}
void fireLauncher() {
  latch2.write(55);
  latch1.write(110);
}
