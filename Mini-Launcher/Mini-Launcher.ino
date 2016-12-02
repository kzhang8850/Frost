/*
 Prototype for Mini Launcher, does Pan and Power calculations
 Uses PID for motor control of Pan, and has user input
 */
#include <math.h>
#include <Servo.h>
//DC Motor Initialization
#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();


//Servo
Servo latch;
//Aiming Motor
Adafruit_DCMotor *panMotor = AFMS.getMotor(4);
Adafruit_DCMotor *powerMotorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *powerMotorRight = AFMS.getMotor(2);


//Power Motor
//Adafruit_DCMotor *PowerMotor1 = AFMS.getMotor(1);
//Adafruit_DCMotor *PowerMotor2 = AFMS.getMotor(2);

//PAN VARIABLES
int panPot = A0;
int panTopSpeed = 100;
float panEncoderValue;    //IN DEGREES
float panOffset = 94.9 + 20; //IN DEGREES
float panError = 0;  //IN DEGREES
float panPGain = 10;
float panDGain = 20;
float panDerivative = 0;
float panProportional = 0;
int panDt = 50;
int panTarget = 0;
int panMotorSpeed = 0;
int panThreshold = 2;
int prevTime = 0;

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
float armOffset = 85+18+5
; //IN DEGREES
bool reset = false;
bool fire = false;
bool arm = false;
int armTarget = 0;

String input;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("starting");
  input = "default";

  //sets up motors
  AFMS.begin();
  
  latch.attach(9);
  fireLauncher();
  panMotor->run(RELEASE);

  while(!reset){
    getSensorData();
    resetLauncher();
  }
  Serial.println("reset");

}

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
    delay(1000);
    reset = false;
    fire = false;
  }
  if(!reset){
    panTarget = 0;
    resetLauncher();
  }
}

void getSensorData() {

  panEncoderValue = voltageToDegrees(analogRead(panPot)) - panOffset;
  Serial.println(panEncoderValue);
  armEncoderValue = voltageToDegrees(analogRead(armPot)) - armOffset;
  //Serial.println(armEncoderValue);
}

float voltageToDegrees(float voltage) {
  // turns raw pot data into degrees
  return (voltage * 220) / 1024;
}

void pan() {
  //Turns target and current encoder value to power output
  if (panTarget > 80) {
    panTarget = 80;
  }
  if (panTarget < -80) {
    panTarget = 80;
  }
  if (millis() - prevTime > panDt) {
    int prevPanEncoderValue = panEncoderValue;
    getSensorData();
    panDerivative = panDGain * (panEncoderValue - prevPanEncoderValue);
    prevTime = millis();
  }
  panError = float(panTarget) - panEncoderValue;
  //Proportional Term of PID
  panProportional = panPGain *  panError;
  //Calculate new motor speed
  panMotorSpeed = panProportional + panDerivative;

  if (panMotorSpeed > panTopSpeed) {
    panMotorSpeed = panTopSpeed;
  }
  if (panMotorSpeed < -panTopSpeed) {
    panMotorSpeed = -panTopSpeed;
  }
  if (abs(panError) < abs(panThreshold)) {
    panMotorSpeed = 0;
  }
  //Serial.println(panError);
  turnpanMotor(panMotorSpeed);
}

void turnpanMotor(int motorSpeed) {
  //Rotates the motors
  if (motorSpeed > 0) {
    panMotor->setSpeed(motorSpeed);
    panMotor->run(BACKWARD);
  } else if (motorSpeed < 0) {
    panMotor->setSpeed(abs(motorSpeed));
    panMotor->run(FORWARD);
  } else {
    panMotor ->run(RELEASE);
  }
}
void turnArmMotors(int motorSpeed) {
  if (motorSpeed > 0) {
    powerMotorLeft->setSpeed(motorSpeed);
    powerMotorRight->setSpeed(motorSpeed);
    powerMotorLeft->run(FORWARD);
    powerMotorRight->run(BACKWARD);
  } else if (motorSpeed < 0) {
    powerMotorLeft->setSpeed(abs(motorSpeed));
    powerMotorRight->setSpeed(abs(motorSpeed));
    powerMotorLeft->run(BACKWARD);
    powerMotorRight->run(FORWARD);
  } else {
    powerMotorLeft->run(RELEASE);
    powerMotorRight->run(RELEASE);
  }
}

void getInput() {
    //INPUT IN FORMAT a = 20, power = 30
    input = Serial.readString();
    if(input == "fire"){
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
    }   

}
void armLauncher() {
  if(armTarget > 50){
    armTarget = 50;
  }
  if(armEncoderValue < armTarget){
    turnArmMotors(100);
  }
  else{
    arm = false;
    turnArmMotors(10);
  }
}


void resetLauncher() {
  if (armEncoderValue > 0){
    turnArmMotors(-60);
  }else{
    turnArmMotors(0);
    reset = true;
    activateLatch();
    delay(100);
  }
}

//Power Functions
void activateLatch() {
  latch.write(90);
}

void fireLauncher() {
  latch.write(170);
}
