/*
 Prototype for Mini Launcher, does Pan and Power calculations
 Uses PID for motor control of Pan, and has user input
 */

#include <math.h>


//DC Motor Initialization
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 


//Aiming Motor
Adafruit_DCMotor *AimMotor = AFMS.getMotor(4);

int baseMotorSpeed = 0;
int SensorPin = A0;
int AngleSensorValue = 0;
int AimMotorSpeed = baseMotorSpeed;



//Power Motor
Adafruit_DCMotor *PowerMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *PowerMotor2 = AFMS.getMotor(2);

int PowerMotorSpeed = 0;
int Encoder = A1;



//Button stuff
int switchValue = 0;
int Switch = 0;





//PID
double iState = 0; //Integrator state
double iMax = 200;
double iMin = -200; //Max and min allowable integrator state

//Gains for pid
double pGain= 1.5;
double iGain= 0.00;
double dGain= 0;

//pid coefficients(terms)
double pTerm;
double iTerm;
double dTerm;

double currentError = 0; //Differences between current and target sensor value
double previousError = 0;

//error threshold
int errorThreshold = 2;

//interval
int dt = 10;
int previousTime = 0;





//angle and power variables
double AimAngle;
int Distance = 0;
double TargetPowerAngle;
int PowerAngle;


//offset
double panOffset = 86.0*1024.0/180.0;
double powerOffset = 94.0*1024.0/180.0;


//used in String parsing input
int separator;
String string1;
String string2;
String ID;
String stringvalue;
int value;
int len;
int PowerError = 0;

//user input for power and angle
String input;



//Determine when to shoot
boolean firing = false;
boolean powered = false;

//Servo stuff
#include <Servo.h>

Servo latch;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  input = "default";
  AimAngle = 0;
  TargetPowerAngle = 0;
  Distance = 0;
  PowerAngle = 0;

  //Servo set up
  latch.attach(9);
  latch.write(170);

  //sets up motors
  AFMS.begin();

  AimMotor->setSpeed(AimMotorSpeed);
  AimMotor->run(FORWARD);
  AimMotor->run(RELEASE);

  PowerMotor1->setSpeed(PowerMotorSpeed);
  PowerMotor1->run(FORWARD);
  PowerMotor1->run(RELEASE);

  PowerMotor2->setSpeed(PowerMotorSpeed);
  PowerMotor2->run(FORWARD);
  PowerMotor2->run(RELEASE);

  pinMode(Switch, INPUT);
  
  resetLauncher();
  delay(500);
  activateLatch();
  
}

void loop() {
  TargetPowerAngle = double(Distance) * 1024.0 /180.0;
  PowerAngle = -1 * analogRead(Encoder);
  PowerAngle = PowerAngle + powerOffset;
//  Serial.println(TargetPowerAngle - PowerAngle);

  if(Serial.available()){
    getInput();
  }
    if(firing != true && powered == true){
      armLauncher(Distance);
      Serial.println("test");
    }
    else if(powered && firing){
      deactivateLatch();
      delay(500);
      PowerMotor1 ->run(RELEASE);
      PowerMotor2 ->run(RELEASE);
      AimMotor->run(RELEASE);
      delay(500);
      resetLauncher();
      delay(200);
      activateLatch();
      
      Serial.println("I have fired.");
      firing = false;
      powered = false;
    }
  
 if (millis() - previousTime > dt){

    AngleSensorValue = analogRead(SensorPin);
    AngleSensorValue = AngleSensorValue - panOffset;
    AngleSensorValue = -1 * AngleSensorValue;
    previousError = currentError;
    currentError = AngleSensorValue - AimAngle;
//    Serial.println(currentError);
    //Launcher not at targeted pan angle
    if (abs(currentError) > errorThreshold) {
      Pan();
      
          
    }
    //Go straight
    else{
      iState = 0;
      AimMotorSpeed = baseMotorSpeed;
      AimMotor->setSpeed(AimMotorSpeed);
      AimMotor->run(RELEASE);
    }
    //Resets interval and prints data
     previousTime = millis();

//    Serial.print("Outputting angle as ");
//    Serial.println((float(AngleSensorValue) * 180) /1024);
    
  }
  
}


void Pan(){
 
      
    //Proportional Term of PID
    pTerm = pGain *  currentError;
    //Differential Term of PID
    dTerm = dGain * (currentError - previousError) / dt;
    //Integral Term of PID
    iState += currentError;

    if (iState > iMax || iState < iMin) {
      iState = 0;
    }
    
    iTerm = iGain * iState * dt;

    //Calculate new motor speed
    AimMotorSpeed = baseMotorSpeed + pTerm + dTerm + iTerm;
    
    if(AimMotorSpeed > 100){
      AimMotorSpeed = 100;
    }
    if(AimMotorSpeed < -100){
      AimMotorSpeed = -100;
    }
    //Gotta turn forward to get back to target angle
    if (AimMotorSpeed > 0){
      AimMotor->setSpeed(abs(AimMotorSpeed));
      AimMotor->run(BACKWARD);

     
    }
    //Gotta turn backward to get back to target angle
    else{
      AimMotor->setSpeed(abs(AimMotorSpeed));
      AimMotor->run(FORWARD);

    }

  
}

void getInput(){

    input = Serial.readString();

    if(input == "fire"){
      firing = true;
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
      
      if(ID == "power"){
        Distance = value;
        powered = true;
        
      }
      else if(ID == "angle"){
        AimAngle = value;
        
      }
      
      separator = string2.indexOf("=");
      len = string2.length()+1;
      ID = string2.substring(0, separator);
      stringvalue = string2.substring(separator+1,len);
      stringvalue.trim();
      value = stringvalue.toInt();
      ID.trim();
      
      
      
      if(ID == "power"){
        Distance = value;
        powered =  true;
        
      }
      else if(ID == "angle"){
        AimAngle = value;
        
      }
      
    }
    else{
      
        separator = input.indexOf("=");
        if (separator != -1){
        ID = input.substring(0, separator);
        stringvalue = input.substring(separator+1, len);
        stringvalue.trim();
        value = stringvalue.toInt();
        ID.trim();
        
        if( ID == "power"){
          Distance = value;
          powered = true;
          
         
        }
        else if(ID == "angle"){
          AimAngle = value;
          
        }
      }
    }
    if(separator != -1){
      AimAngle = (AimAngle * 1024)/180;
      Serial.print("Angle set to: ");
      Serial.println(AimAngle);
      Serial.print("Distance set to: ");
      Serial.println(Distance);
      delay(1000);
    }
}





//Power Functions
void activateLatch(){
  latch.write(60);
  delay(15);
  PowerMotorSpeed = 0;
  PowerMotor1 ->setSpeed(PowerMotorSpeed);
  PowerMotor1 ->run(RELEASE);
  PowerMotor2 ->setSpeed(PowerMotorSpeed);
  PowerMotor2 ->run(RELEASE);
}

void armLauncher(int distance){
//  int velocity = 10;   ///////////THIS IS HARDCODED
  TargetPowerAngle = (double(distance) * 1024.0) /180.0;

  PowerAngle = -1 * analogRead(Encoder);
  PowerAngle = PowerAngle + powerOffset;
  //Serial.println(PowerError);
  //PowerError = TargetPowerAngle - PowerAngle;
  //PowerMotorSpeed = PowerError;
  PowerMotorSpeed = 127;
  if (PowerMotorSpeed > 127){
    PowerMotorSpeed = 127;
  }else if(PowerMotorSpeed <= 0){
    PowerMotorSpeed = 0;
  }
  if(PowerAngle < TargetPowerAngle){
    PowerMotor1 ->setSpeed(PowerMotorSpeed);
    PowerMotor1 ->run(FORWARD);
    PowerMotor2 ->setSpeed(PowerMotorSpeed);
    PowerMotor2 ->run(BACKWARD);
    //Serial.println("run motors");
  }else{
    firing = true;
    PowerMotor1 ->setSpeed(15);
    PowerMotor1 ->run(FORWARD);
    PowerMotor2 ->setSpeed(15);
    PowerMotor2 ->run(BACKWARD);
  }
  
}

void deactivateLatch(){
  latch.write(170);
  //PowerMotorSpeed = 0;
  //PowerMotor1 ->setSpeed(PowerMotorSpeed);
  //PowerMotor1 ->run(FORWARD);
  //PowerMotor2 ->setSpeed(PowerMotorSpeed);
  //PowerMotor2 ->run(BACKWARD);
  
}

void resetLauncher(){
  PowerAngle = -1 * analogRead(Encoder);
  PowerAngle = PowerAngle + powerOffset;
  AimAngle = 0;
  while(PowerAngle > 0){
    PowerAngle = -1 * analogRead(Encoder);
    PowerAngle = PowerAngle + powerOffset;
    PowerMotorSpeed = 60;
    PowerMotor1 ->setSpeed(PowerMotorSpeed);
    PowerMotor1 ->run(BACKWARD);
    PowerMotor2 ->setSpeed(PowerMotorSpeed);
    PowerMotor2 ->run(FORWARD);
    
  }
  PowerMotorSpeed = 0;
  PowerMotor1 ->setSpeed(PowerMotorSpeed);
  PowerMotor1 ->run(RELEASE);
  PowerMotor2 ->setSpeed(PowerMotorSpeed);
  PowerMotor2 ->run(RELEASE);

  
}


