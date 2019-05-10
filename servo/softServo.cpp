#include "softServo.h"

softServo::softServo() {

  this->pinsInitialized = false;

}

softServo::softServo(int enablePin, int aPin, int bPin, int i1pin, int i2pin, float kp, float ki, float kd, bool motorReversed) {

  setup(enablePin, aPin, bPin, i1pin, i2pin, kp, ki, kd, motorReversed);

}

void softServo::setup(int enablePin, int aPin, int bPin, int i1pin, int i2pin, float kp, float ki, float kd, bool motorReversed) {

  this->enablePin = enablePin;
  this->aPin = aPin;
  this->bPin = bPin;
	this->i1pin = i1pin;
	this->i2pin = i2pin;
  this->motorReversed = motorReversed;

  pinMode(enablePin, OUTPUT);
  pinMode(aPin, OUTPUT);
  pinMode(bPin, OUTPUT);
  pinMode(i1pin, INPUT);
  pinMode(i2pin, INPUT);

  this->pinsInitialized = true;

	readPos(); // Update quad1 and quad2
	this->pos = 0;

  setPids(kp, ki, kd); //Also clears I buffer

  setPower(0); 

}

void softServo::setPids(float kp, float ki, float kd) { //Reset the pid tuning

  this->kp = kp;
  this->ki = ki;
  this->kd = kd;

  for (int i = 0; i < winLen; i++) this->window[i] = 0; //re-init integral
  this->winPos = 0;
  this->total = 0;
}

void softServo::setPower(int power) {

  this->power = power;
  this->isPos = false;

}

void softServo::setPos(int goalPos) {


  if (!isPos) { //If switching from power to position, re-init integral
    for (int i = 0; i < winLen; i++) window[i] = 0;
    winPos = 0;
    total = 0;
  }

  this->goalPos = goalPos;
  isPos = true;

}

int softServo::getPos() {

  return this->pos;

}

void softServo::readPos() {
	bool s1 = digitalRead(i1pin) == HIGH;
	bool s2 = digitalRead(i2pin) == HIGH;

	if (this->quad1 != s1) {
		if (this->quad2 != s2) {
			Serial.println("Can't keep up!");
		}
		(s1 == s2) ? this->pos++ : this->pos--;
	}
	if (this->quad2 != s2) {
		(s1 != s2) ? this->pos++ : this->pos--;
	}

	this->quad1 = s1;
	this->quad2 = s2;

}

void softServo::update() {

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
    int d = float(error - lastError) / (float(loopTime) * kd);

    //Serial.println("P: " + String(p) + " I: " + String(i) + "D: " + String(d) );
    //Serial.println("Last error: " + String(lastError) + " Current error: " + String(error) + " Last loop time: " + String(loopTime));

    power = p + i + d;

    lastError = error;

  } else {
		power = this->power;
	}

  if (motorReversed) power = -power;

  if (power > 0) power += deadSpot;
  else if (power < 0) power -= deadSpot;

  if (power > maxPower) power = maxPower;
  if (power < -maxPower) power = -maxPower;

	this->power = power;
  analogWrite(enablePin, abs(power));
  digitalWrite(aPin, (power>0) ? HIGH : LOW);
  digitalWrite(bPin, (power>0) ? LOW : HIGH);

  loopTime = micros() - lastTime; //For decrementing turn timer
  lastTime = micros(); //So we'll know how long the next loop takes


}
