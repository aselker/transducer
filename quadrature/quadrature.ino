
int motor1 = PA1;
int motor2 = PA2;
int maxPower = 60;

int in1 = PA6;
int in2 = PA7;
bool s1, s2, l1, l2;
long int count = 0;

void setup() {
	pinMode(motor1, OUTPUT);
	pinMode(motor2, OUTPUT);
	digitalWrite(motor1, LOW);
	digitalWrite(motor2, LOW);

	pinMode(in1, INPUT);
	pinMode(in2, INPUT);
	s1 = l1 = digitalRead(in1);
	s2 = l2 = digitalRead(in2);

	Serial.begin(115200);
}

void loop() {

	s1 = digitalRead(in1) == HIGH;
	s2 = digitalRead(in2) == HIGH;

	if (l1 != s1 && l2 != s2) Serial.println("Can't keep up!");

	if (l1 != s1) {
		(s1 == s2) ? count++ : count--;
	}
	if (l2 != s2) {
		(s1 != s2) ? count++ : count--;
	}

	Serial.println(count);
		
	l1 = s1;
	l2 = s2;
}
