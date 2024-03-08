#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

/* This program prints out the tilt angle, without offset, or the angular velocity to the serial monitor.
	Press enter when prompted to begin printing values every 10 ms. Stop monitoring when you wish to no longer record.
	These values can then be copied and analyzed in some other program of choice.
*/

/* IMU wiring:
	GND to GND - black wire
	5V to 5V - red wire
	SDA to SDA - white wire
	SCL to SCL - orange wire
*/

void setup() {
	Serial.begin(9600);

	/* Initialise the sensor */
	if(!bno.begin()) {
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while(true); // loops forever, reset manually
	}

	bno.setExtCrystalUse(true);

	Serial.println("Press enter to begin recording");
	while (Serial.available() == 0);
	Serial.read();
	Serial.flush();
}

void loop() {
	/* Uncomment this line to print out tilt angle readings */
	// Serial.println(degrees(bno.getQuat().toEuler()[1]));

	/* Uncomment this line to print out angular velocity readings */
	// Serial.println(degrees(bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).z()));

	delay(10);
}