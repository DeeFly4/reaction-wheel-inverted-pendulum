#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MotorShield.h>
#include <functions.h>

#define angle_increment 0.004 // 0.004, working range ~[0.003-0.005]

extern float Kp, Ki, Kd, Kc;
extern float theta, controlSignal, ref, error;

extern Adafruit_DCMotor *motor;
extern Adafruit_BNO055 bno;

int16_t speed;

bool vertical;
bool dithering = true; // dithering is helpful with good angle_increment

float calibrateOffset() {
	Serial.println("Balance the rig for calibration, tilt setup to the right");
	while (Serial.available() == 0); // waits for user to hit enter
	Serial.read();
	Serial.flush();

	float angleMin = bno.getQuat().toEuler()[1]; // angle in radians
	Serial.print("angleMin = ");
	Serial.println(degrees(angleMin));

	Serial.println("Balance the rig for calibration, tilt setup to the left");
	while (Serial.available() == 0); // waits for user to hit enter
	Serial.read();
	Serial.flush();

	float angleMax = bno.getQuat().toEuler()[1]; // angle in radians
	Serial.print("angleMax = ");
	Serial.println(degrees(angleMax));

	return (angleMax + angleMin) / 2;
}

void printSignals() {
	Serial.print("theta = ");
	Serial.print(theta);
	Serial.print("\tu = ");
	Serial.print(controlSignal);
	Serial.print("\tref = ");
	Serial.println(ref);
}

void printPIDGains() {
	Serial.print("Kp = ");
	Serial.print(Kp);
	Serial.print("\tKi = ");
	Serial.print(Ki);
	Serial.print("\tKd = ");
	Serial.print(Kd);
	Serial.print("\tKc = ");
	Serial.print(Kc);
	Serial.print("\tdithering ");
	if (dithering) {
		Serial.println("on");
	} else {
		Serial.println("off");
	}
}

void tuning() {
	if (!Serial.available()) return;
	delay(2);
	char param = Serial.read();
	if (!Serial.available()) return;
	char cmd = Serial.read();
	Serial.flush();
	switch(param) {
		case 'p':
			if (cmd == '+') Kp += 2.5;
			if (cmd == '-') Kp -= 2.5;
			break;
		case 'i':
			if (cmd == '+') Ki += 0.1;
			if (cmd == '-') Ki -= 0.1;
			break;
		case 'd':
			if (cmd == '+') Kd += 10;
			if (cmd == '-') Kd -= 10;
			break;
		case 'c':
			if (cmd == '+') Kc += .05;
			if (cmd == '-') Kc -= .05;
			break;
		case 't':
			dithering = !dithering;
			ref = 0;
	}
	printPIDGains();
}

void spinMotor() {
	speed = (int16_t) round(controlSignal);
	
	if (abs(speed) < 25) { // The motor appears to not spin at all if speed < ~25
		motor->setSpeed(25);
	} else {
		motor->setSpeed((uint8_t) abs(speed));
	}

	if (speed > 0) {
		motor->run(BACKWARD);
	} else {
		motor->run(FORWARD);
	}
}

void dither() {
	if (dithering) {
		if (error < 0) {
			ref -= angle_increment;
		} else if (error > 0) {
			ref += angle_increment;
		}
	}
}

bool isVertical() {
	if (abs(error) < 1) {
		vertical = true;
	} else if (abs(error) > 15) {
		vertical = false;
	}
	return vertical;
}