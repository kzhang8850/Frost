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
int AngleSensorValue;
int AimMotorSpeed = baseMotorSpeed;



//Power Motor
Adafruit_DCMotor *PowerMotor = AFMS.getMotor(3);

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
int errorThreshold = 2;

//interval
int dt = 10;
int previousTime = 0;





//angle and power variables
double AimAngle;
int Distance;
int TargetPowerAngle;
int PowerAngle;




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


//Determine when to shoot
boolean firing = false;

//Servo stuff
#include <Servo.h>

Servo latch;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  input = "default";
  AimAngle = 0;
  TargetPowerAngle = 0;
  Distance = 10;
  PowerAngle = 0;

  //Servo set up
 // latch.attach(9);
 // latch.write(0);

  //sets up motors
  AFMS.begin();

  AimMotor->setSpeed(AimMotorSpeed);
  AimMotor->run(FORWARD);
  AimMotor->run(RELEASE);

//  PowerMotor->setSpeed(PowerMotorSpeed);
//  PowerMotor->run(FORWARD);
//  PowerMotor->run(RELEASE);


//  pinMode(Switch, INPUT);
  
//  resetLauncher();
  
}

void loop() {

//  switchValue = digitalRead(Switch);
//  if(switchValue == HIGH){
//    activateLatch();
//  }

  if(Serial.available()){
    getInput();
//
//    if(firing == true && abs(currentError) <= errorThreshold){
//      armLauncher(Distance);
//      deactivateLatch();
//      delay(1000);
//      resetLauncher();
//      
//      Serial.println("I have fired.");
//      firing = false;
//    }
  }

  
 if (millis() - previousTime > dt){

    AngleSensorValue = analogRead(SensorPin);
    previousError = currentError;
    currentError = AngleSensorValue - AimAngle;
  
    //Launcher not at targeted pan angle
    if (abs(currentError) > errorThreshold) {
      Pan();
          
    }
    //Go straight
    else{
      AimMotorSpeed = baseMotorSpeed;
      AimMotor->setSpeed(AimMotorSpeed);
      AimMotor->run(RELEASE);
    }
    //Resets interval and prints data
     previousTime = millis();

    Serial.print("Outputting angle as ");
    Serial.println((float(AngleSensorValue) * 180) /1024);
    
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
    
    if(AimMotorSpeed > 255){
      AimMotorSpeed = 255;
    }
    if(AimMotorSpeed < -255){
      AimMotorSpeed = -255;
    }

    //Gotta turn forward to get back to target angle
    if (AimMotorSpeed > 0){
      AimMotor->setSpeed(abs(AimMotorSpeed));
      AimMotor->run(FORWARD);

     
    }
    //Gotta turn backward to get back to target angle
    else{
      AimMotor->setSpeed(abs(AimMotorSpeed));
      AimMotor->run(BACKWARD);

    }

  
}

void getInput(){

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
        Distance = value;
        firing = true;
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
        firing = true;
      }
      else if(ID == "angle"){
        AimAngle = value;
        
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
        Distance = value;
        firing = true;
       
      }
      else if(ID == "angle"){
        AimAngle = value;
        
      }
    }

    AimAngle = (AimAngle * 1024)/180;
    Serial.print("Angle set to: ");
    Serial.println(AimAngle);
    Serial.print("Distance set to: ");
    Serial.println(Distance);
    delay(1000);
}



//Power Functions
void activateLatch(){
  latch.write(90);
  delay(15);
  PowerMotorSpeed = 0;
  PowerMotor ->setSpeed(PowerMotorSpeed);
  PowerMotor ->run(RELEASE);
}

void armLauncher(int distance){
  int velocity = 10;   ///////////THIS IS HARDCODED
  TargetPowerAngle = .5 * asin(9.81 * pow(distance, 2) / velocity);

  PowerAngle = analogRead(Encoder);
  while(PowerAngle < TargetPowerAngle){
    PowerAngle = analogRead(Encoder);
    PowerMotorSpeed = 50;
    PowerMotor ->setSpeed(PowerMotorSpeed);
    PowerMotor ->run(FORWARD);
  }

  
}

void deactivateLatch(){
  latch.write(0);
  delay(15);
  PowerMotorSpeed = 5;
  PowerMotor ->setSpeed(PowerMotorSpeed);
  PowerMotor ->run(FORWARD);
  
}

void resetLauncher(){
  PowerAngle = analogRead(Encoder);
  while(PowerAngle > 0){
    PowerAngle = analogRead(Encoder);
    PowerMotorSpeed = 15;
    PowerMotor ->setSpeed(PowerMotorSpeed);
    PowerMotor ->run(BACKWARD);
  }
  PowerMotorSpeed = 0;
  PowerMotor ->setSpeed(PowerMotorSpeed);
  PowerMotor ->run(RELEASE);

  
}


