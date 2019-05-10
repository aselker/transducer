#include "softServo.h"

softServo::softServo() {

  this->pinsInitialized = false;

}

softServo::softServo(int enablePin, int aPin, int bPin, int potPin, float kp, float ki, float kd, bool motorReversed, bool potReversed) {

  setup(enablePin, aPin, bPin, potPin, kp, ki, kd, motorReversed, potReversed);

}

void softServo::setup(int enablePin, int aPin, int bPin, int potPin, float kp, float ki, float kd, bool motorReversed, bool potReversed) {

  this->enablePin = enablePin;
  this->aPin = aPin;
  this->bPin = bPin;
  this->potPin = potPin;
  this->motorReversed = motorReversed;
  this->potReversed = potReversed;

  pinMode(enablePin, OUTPUT);
  pinMode(aPin, OUTPUT);
  pinMode(bPin, OUTPUT);
  pinMode(potPin, INPUT);

  this->pinsInitialized = true;

  setPids(kp, ki, kd); //Also clears I buffer

  setPower(0); 

}

void softServo::setPids(float kp, float ki, float kd) { //Reset the pid tuning

  this->kp = kp;
  this->ki = ki;
  this->kd = kd;

  for (int i = 0; i < winLen; i++) window[i] = 0; //re-init integral
  winPos = 0;
  total = 0;
}

void softServo::setPower(int power) {

  this->power = power;
  isPos = false;

}

void softServo::setPos(int pos) {


  if (!isPos) { //If switching from power to position, re-init integral
    for (int i = 0; i < winLen; i++) window[i] = 0;
    winPos = 0;
    total = 0;
  }

  this->goalPos = pos;
  isPos = true;

}

int softServo::getPos() {

  return pos;

}

void softServo::update() {

  pos = potReversed ? posRange-analogRead(potPin) : analogRead(potPin);

  if (isPos) {

    int error = pos - goalPos;

    winPos++; //Go to the next position in the buffer
    winPos = winPos % winLen;
    total += error; //Add the value being added
    total -= window[winPos]; //Remove the value being removed
    window[winPos] = error; //Insert the new value, knocking out an older one

    float avg = total / winLen;

    int p = error * kp;
    int i = avg * ki;
    int d = float(error - lastError) / float(loopTime) * kd;

    //Serial.println("P: " + String(p) + " I: " + String(i) + "D: " + String(d) );
    //Serial.println("Last error: " + String(lastError) + " Current error: " + String(error) + " Last loop time: " + String(loopTime));

    power = (p + i + d) * (255.0/float(posRange));

    lastError = error;

  } //End if isPos


  if (motorReversed) power = -power;

  if (power > 0) power += deadSpot;
  else if (power < 0) power -= deadSpot;

  if (power > maxPower) power = maxPower;
  if (power < -maxPower) power = -maxPower;

  analogWrite(enablePin, abs(power));
  digitalWrite(aPin, (power>0) ? HIGH : LOW);
  digitalWrite(bPin, (power>0) ? LOW : HIGH);

  loopTime = millis() - lastTime; //For decrementing turn timer
  lastTime = millis(); //So we'll know how long the next loop takes


}
