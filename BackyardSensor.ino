/*
Name:		BackyardSensor.ino
Created:	12/06/2015 7:49:07 PM
Author:		Tim Lampman
*/

#include <Wire.h>
#include <LumSensor.h>
#include <HTU21D.h>
#include <MAX1704.h>
#include <XBee.h>
#include <AltSoftSerial.h>
#include <JeeLib.h>

// Sleep/Delay behaviour
ISR(WDT_vect) { Sleepy::watchdogEvent(); } // Setup the watchdog

const bool prefer_sleep = false;		// Specifies whether to use delay (false) or sleep (true) for long delays without sensor power
const bool high_power = true;		// Specifies whether to use delay (true) or sleep (false) for periods when power supplied to sensors

// XBee variables
AltSoftSerial xbeeSerial;  // The software serial port for communicating with the Xbee (TX Pin 9, RX Pin 8)
XBee localRadio = XBee();  // The connection for the local coordinating radio

// XBee command codes
const uint8_t CMD_SENSOR_DATA = 4;

// XBee data codes
const uint8_t TEMPERATURE_CODE = 1;
const uint8_t LUMINOSITY_CODE = 2;
const uint8_t PRESSURE_CODE = 3;
const uint8_t HUMIDITY_CODE = 4;
const uint8_t POWER_CODE = 5;
const uint8_t LUX_CODE = 6;
const uint8_t HEATING_CODE = 7;
const uint8_t THERMOSTAT_CODE = 8;
const uint8_t TEMP_12BYTE_CODE = 9;
const uint8_t BATTERY_SOC_CODE = 10;

// Timing variables
const unsigned long SENSOR_DELAY = 60000;	// The sleep period (ms)
const unsigned long DELAY_PERIODS = 1;		// The number of sleep periods before updating sensor readings
const unsigned long STARTUP_DELAY = 3000;	// The period allowed for component warmup and initialization (ms).
const unsigned long COMM_DELAY = 1000;		// The period allowed for XBee communications to initialize/finalize (ms).

// Local pins
const int POWER_PIN = 12;	// The digital pin for powering the sensors and XBee

// Sensor objects
HTU21D airSensor;
MAX1704 batterySensor;
AutoLightSensor lightSensor;

// Union for conversion of numbers to byte arrays
union FloatConverter {
	float f;
	uint8_t b[sizeof(float)];
};

void SmartDelay(unsigned long delay_time, bool force_delay);
void Message(String msg);

//=============================================================================
// SETUP
//=============================================================================
void setup() {
	// Initialize the pins
	pinMode(POWER_PIN, OUTPUT);		// Set the power pin to digital output
	digitalWrite(POWER_PIN, HIGH);	// Turn on the power

	// Start the I2C interface
	lightSensor.begin();
	
	// Setup the serial communications
	Serial.begin(9600);
	Message("Starting Serial...");
	delay(1000);

	// Start the power monitor
	Message("Setting up battery monitor");
	batterySensor.reset();		// Do a power reset
	batterySensor.quickStart();	// Quickly assess the state of charge (SOC)
}

//=============================================================================
// MAIN LOOP
//=============================================================================
void loop() {
	//-------------------------------------------------------------------------
	// POWER UP COMPONENTS
	//-------------------------------------------------------------------------
	// Turn on the power to the attached components
	Message("Powering up components...");
	digitalWrite(POWER_PIN, HIGH);
	SmartDelay(STARTUP_DELAY, high_power);	// Warmup delay
	Message("Awakening battery sensor");
	batterySensor.awake();	// Wake the battery sensor

	// Connect to the XBee
	Message("Starting XBee connection...");
	xbeeSerial.begin(9600);
	localRadio.setSerial(xbeeSerial);
	Message("FINISHED");

	// Delay while components power up
	SmartDelay(COMM_DELAY, high_power);

	//-------------------------------------------------------------------------
	// COLLECT SENSOR DATA
	//-------------------------------------------------------------------------
	// Read the luminosity
	Message("Reading the luminosity");
	FloatConverter Luminosity;
	Luminosity.f = lightSensor.getLuminosity();

	// Read the temperature in the box - could be used for humidity corrections
	Message("Reading tempeature");
	FloatConverter Temperature;
	Temperature.f = airSensor.readTemperature();

	// Read the humidity
	Message("Reading humidity");
	FloatConverter Humidity;
	float raw_humidity = airSensor.readHumidity();
	//	Humidity.f = raw_humidity - 0.15*(25.0 - Temperature.f);	// Correction for HTU21D from spec sheet
	Humidity.f = raw_humidity;

	// Read the battery voltage
	Message("Reading battery voltage");
	FloatConverter Power;
	Power.f = batterySensor.cellVoltage();

	// Read the battery SOC
	Message("Reading battery state of charge");
	FloatConverter SOC;
	SOC.f = batterySensor.stateOfCharge();

	// Sleep the battery sensor
	Message("Putting battery sensor to sleep");
	batterySensor.sleep();

	//-------------------------------------------------------------------------
	// SEND DATA THROUGH XBEE
	//-------------------------------------------------------------------------
	// Create the byte array to pass to the XBee
	Message("Creating XBee data transmission");
	size_t floatBytes = sizeof(float);
	uint8_t package[1 + 5 * (floatBytes + 1)];
	package[0] = CMD_SENSOR_DATA;
	package[1] = TEMPERATURE_CODE;
	package[1 + (floatBytes + 1)] = HUMIDITY_CODE;
	package[1 + 2 * (floatBytes + 1)] = POWER_CODE;
	package[1 + 3 * (floatBytes + 1)] = BATTERY_SOC_CODE;
	package[1 + 4 * (floatBytes + 1)] = LUX_CODE;
	for(int i = 0; i < floatBytes; i++) {
		package[i + 2] = Temperature.b[i];
		package[i + 2 + (floatBytes + 1)] = Humidity.b[i];
		package[i + 2 + 2 * (floatBytes + 1)] = Power.b[i];
		package[i + 2 + 3 * (floatBytes + 1)] = SOC.b[i];
		package[i + 2 + 4 * (floatBytes + 1)] = Luminosity.b[i];
	}

	// Create the message text for debugging output
	String xbee_message = "Sent the following message(" + String(Temperature.f) + "," + Humidity.f + "," + Power.f + "," + SOC.f + "," + Luminosity.f + "): ";
	for(int i = 0; i < sizeof(package); i++) {
		if(i != 0) xbee_message += "-";
		xbee_message += String(package[i], HEX);
	}
	Message(xbee_message);

	// Send the data package to the coordinator through the XBee
	Message("Sending XBee transmission");
	XBeeAddress64 address = XBeeAddress64(0x00000000, 0x00000000);
	ZBTxRequest zbTX = ZBTxRequest(address, package, sizeof(package));
	localRadio.send(zbTX);

	// Transmission delay
	Message("Pause while XBee message sent");
	SmartDelay(COMM_DELAY, high_power);

	//-------------------------------------------------------------------------
	// POWER DOWN COMPONENTS AND WAIT FOR NEXT CYCLE
	//-------------------------------------------------------------------------
	// Turn off the power to the components and sleep
	Message("Turn off power to components");
	xbeeSerial.end();	// Turn off the serial communication with the xbee
//	digitalWrite(POWER_PIN, LOW);

	// Cycle delay
	Message("Sensor delay");
	for(int i = 0; i < DELAY_PERIODS; i++) SmartDelay(SENSOR_DELAY, false);
}

//=============================================================================
// SmartDelay
//=============================================================================
void SmartDelay(unsigned long delay_time, bool force_delay) {
	if(force_delay) delay(delay_time);	// the user is selecting a delay and not sleeping
	else {
		if(prefer_sleep) Sleepy::loseSomeTime(delay_time);
		else delay(delay_time);
	}
}

//=============================================================================
// Message
//=============================================================================
void Message(String msg) {
	Serial.print(millis());
	Serial.print(": ");
	Serial.println(msg);
}
