#include "Servo.h"
#include "softServo.h"

const int ampPin = PB0;
const int ampMin = 56;
const int ampMax = 117;

const int enablePin = PA3;
const int motor1 = PA1;
const int motor2 = PA2;

const int in1 = PA6;
const int in2 = PA7;

const float kp = 15, ki= 0, kd = 0;

softServo freqMotor;
Servo ampMotor;
int pos = 0;
String input = "";

void setup() {

	freqMotor.setup(enablePin, motor1, motor2, in1, in2, kp, ki, kd, false);
	ampMotor.attach(ampPin);
	ampMotor.write((ampMin + ampMax)/2);

	Serial.begin(115200);
	Serial.setTimeout(100);
}

void loop() {

	while (Serial.available()) {
		char in = (char)Serial.read();
		if (in == '\n') {
			int pos = (input.toFloat() * (ampMax - ampMin) + ampMin);
			if (pos > ampMax) pos = ampMax;
			if (pos < ampMin) pos = ampMin;
			ampMotor.write(pos);
			input = "";
		} else {
			input += in;
		}
	}

	freqMotor.setPos(4 * millis());

	freqMotor.readPos();
	freqMotor.update();

}
