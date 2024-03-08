#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_MS_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <imumaths.h>
#include <functions.h>

/*
	IMU wiring:
		GND to GND - black wire
		5V to 5V - red wire
		SDA to SDA - white wire
		SCL to SCL - orange wire

	Motor wiring:
		+ on motor -(red wire)- Outer port on shield
		Other on motor -(black wire)- Inner port on shield

	Power source: (VERY IMPORTANT)
		Battery or power supply - remove VIn jumper
		12V DC barrel jack - put on VIn jumper
*/

#define MAIN_LOOP_MILLIS 10 // 10 is lower limit for IMU
#define PRINT_LOOP_MILLIS 250 // no need to print every sample
#define SAMPLING_FREQUENCY 100
#define dt 0.01
#define Imax 127 // 127, ~[100 150]

/* Time variables to keep track of loop timings (milliseconds) */
long t, prevPrint, prevLoop;

/* Control variables */
float theta, I, D, controlSignal, error, prevError;

/* Setpoint and gains on startup, best working values so far in comments */
float ref = 0;
float Kp = 45; // 45, ~[45, 55]
float Ki = 120 * dt; // 120, ~[100, 175]
float Kd = 2.5 * SAMPLING_FREQUENCY; // 2.5, ~[2, 3]
float Kc = -0.1; // -0.1, working range is quite wide, ~[-0.4, -0.05]

/* Angle offset obtained from calibration during setup() */
float angleOffset;

/* Create motor shield object with default I2C address */
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

/* Create motor object. Select which 'port' M1, M2, M3 or M4. In this case, M1 */ 
Adafruit_DCMotor *motor = AFMS.getMotor(1);

/* Create IMU object with default I2C adress (default address is 0x29 or 0x28)
									  id, address */
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
	Serial.begin(9600);

	/* Initialize Motor Shield object with the default frequency 1.6KHz */
	if (!AFMS.begin()) {
		Serial.println("Could not find Motor Shield. Check wiring.");
		while(true); // loops forever, reset manually
	}
	Serial.println("Motor Shield found");

	/* Initialize the IMU sensor */
	if(!bno.begin()) {
		Serial.print("Could not find IMU. Check wiring");
		while(true); // loops forever, reset manually
	}
	Serial.println("IMU found");
	bno.setExtCrystalUse(true);

	// turn on motor (is this needed?)
	motor->setSpeed(100); // dummy value
	motor->run(FORWARD);
	motor->run(RELEASE);

	angleOffset = calibrateOffset();
	Serial.print("angleOffset = ");
	Serial.println(angleOffset);

	printPIDGains();

	Serial.println("Hit enter to start controller");
	while (Serial.available() == 0); // waits for user to hit enter
	Serial.read();
	Serial.flush();
}

void loop() {
	t = millis(); // get current time

	/* Tunes control parameters if there is input from serial */
	tuning();
	
	/* The controller acts at regular intervals */
	if (t - prevLoop < MAIN_LOOP_MILLIS) return; // skip if not enough time has passed

	/* Get sensor reading for angle */
	theta = degrees(bno.getQuat().toEuler()[1] - angleOffset);
	error = ref - theta;

	/* If pendulum is not close to vertical, do not control */
	if (!isVertical()) {
		motor->run(RELEASE);
		ref = 0;
		I = 0;
		controlSignal = 0;
		prevError = 0;
		prevLoop = t;
		return;
	}

	/* Shifts reference further away by small amount if there is error, can be toggled */
	dither();

	/* Compute integral part and saturate */
	I = constrain(I + error, -Imax, Imax);

	/* Compute derivative estimate */
	D = (error - prevError);

	/* Compute control signal and saturate */
	controlSignal = constrain(Kp * error + Ki * I + Kd * D + Kc * controlSignal, -255, 255);
	
	spinMotor();

	prevError = error;

	/* Print some signals of interest at regular intervals */
	if (t - prevPrint >= PRINT_LOOP_MILLIS) {
		printSignals();
		prevPrint = t;
	}

	prevLoop = t;
}