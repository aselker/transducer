#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(3);
Servo amplitudeServo;
// Pin Definition
#define outputA 3
#define outputB 4
#define powerRail 8
#define servoPin 5
#define servoMin 56
#define servoMax 117
// Variable Instatiation
int maxSpeed = 100;
bool aLastState, bLastState, aState, bState;
int counter = 0;
float sineValue;

void setup() {
  Serial.begin(115200);
  pinMode (outputA,INPUT);
  pinMode (outputB,INPUT);
  pinMode (servoPin,OUTPUT);
  pinMode (powerRail,OUTPUT);
  digitalWrite(powerRail,HIGH);
  aLastState = aState = digitalRead(outputA);
  bLastState = bState = digitalRead(outputB);
  amplitudeServo.attach(servoPin);
  AFMS.begin();  // create with the default frequency 1.6KHz OR with a different frequency, say 1000Hz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(maxSpeed);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
    uint8_t i;
  myMotor->run(FORWARD);
  for (i=0; i<maxSpeed; i++) {
    myMotor->setSpeed(i);  
    delay(10);
  }
  
}

void loop() {
  // amplitudeServo.write((float(servoMax-servoMin)*0.5*sin(float(millis())/(628.0*2.0)))+((servoMax+servoMin)/2.0));
  aState = digitalRead(outputA);
  bState = digitalRead(outputB);
  if (aState != aLastState){
    if (bState != bLastState) {
      Serial.println("Can't keep up!");
    }
    if(bState != aState){
      counter ++;
    } else {
      counter --;
    }
    //Serial.println(counter);
  }
   aLastState = aState;
   bLastState = bState;
}
