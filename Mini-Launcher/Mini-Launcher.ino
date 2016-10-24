/*
 Prototype for Mini Launcher, does Pan and Power calculations
 Uses PID for motor control
 */
 
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *Motor = AFMS.getMotor(4);


int baseMotorSpeed = 0;

double iState = 0; //Integrator state
double iMax = 200;
double iMin = -200; //Max and min allowable integrator state

//Gains for pid
double pGain= 2;
double iGain=0;
double dGain=0;

//pid coefficients(terms)
double pTerm;
double iTerm;
double dTerm;

double currentError = 0; //Differences between current and target sensor value
double previousError = 0;

//error threshold
int errorThreshold = 1;

//Sensor initialiizations
int SensorPin = A0;
int SensorValue;

//motor speed
int motorSpeed = baseMotorSpeed;

//interval
int dt = 10;
int previousTime = 0;

//angle and power variables
double angle;
int power;

//used in String parsing input
int separator;
String string1;
String string2;
String ID;
String stringvalue;
int value;
int len;

//user input for power and angle
String input;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  input = "default";
  angle = 0;
  power = 10;

  //sets up motors
  AFMS.begin();

  Motor->setSpeed(baseMotorSpeed);
  Motor->run(FORWARD);
  Motor->run(RELEASE);
  
}

void loop() {
   //time interval
  if (millis() - previousTime > dt){
  
    SensorValue = analogRead(SensorPin);
    previousError = currentError;
    currentError = SensorValue - angle;
  
    //One of sensor detecting a tape
    if (abs(currentError) > errorThreshold) {
      
      //Proportional Term of PID
      pTerm = pGain * abs(currentError);
      //Differential Term of PID
      dTerm = dGain * (abs(currentError) - abs(previousError)) / dt;
      //Integral Term of PID
      iState += currentError;

      if (iState > iMax || iState < iMin) {
        iState = 0;
      }
      
      iTerm = iGain * iState * dt;

      //Calculate new motor speed
      motorSpeed = baseMotorSpeed + pTerm + dTerm + iTerm;
      if(motorSpeed > 255){
        motorSpeed = 255;
      }
      //Left sensor detecting tape, turn left
      if (motorSpeed > 0){
        Motor->setSpeed(motorSpeed);
        Motor->run(FORWARD);

       
      }
      //Right sensor detecting tape, turn right
      else{
        Motor->setSpeed(motorSpeed);
        Motor->run(BACKWARD);

      }
    }
    //Go straight
    else{
      motorSpeed = baseMotorSpeed;
      Motor->setSpeed(motorSpeed);
      Motor->run(FORWARD);
    }
    //Resets interval and prints data
    previousTime = millis();
  }
  Serial.print("Outputting angle as ");
  Serial.println((float(SensorValue) * 180) /1024);
  

}

void serialEvent(){
  if(Serial.available()){
    input = Serial.readString();
    
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
      
      if(ID == "power"){
        power = value;
      }
      else if(ID == "angle"){
        angle = value;
      }
      
      separator = string2.indexOf("=");
      len = string2.length()+1;
      ID = string2.substring(0, separator);
      stringvalue = string2.substring(separator+1,len);
      stringvalue.trim();
      value = stringvalue.toInt();
      ID.trim();
      
      
      
      if(ID == "power"){
        power = value;
      }
      else if(ID == "angle"){
        angle = value;
      }
      
    }
    else{
      
      separator = input.indexOf("=");
      ID = input.substring(0, separator);
      stringvalue = input.substring(separator+1, len);
      stringvalue.trim();
      value = stringvalue.toInt();
      ID.trim();
      
      if( ID == "power"){
        power = value;
       
      }
      else if(ID == "angle"){
        angle = value;
      }
    }

    angle = (angle * 1024)/180;
    Serial.print("Angle set to: ");
    Serial.println(angle);
    Serial.print("Power set to: ");
    Serial.println(power);
    delay(1000);
  }
}

