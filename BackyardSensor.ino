/*
Name:		BackyardSensor.ino
Created:	12/06/2015 7:49:07 PM
Author:		Tim Lampman
*/

#include <OneWire.h>
#include <Wire.h>
#include <LumSensor.h>
#include <HTU21D.h>
#include <MAX1704.h>
#include <XBee.h>
#include <AltSoftSerial.h>
#include <JeeLib.h>

// Sleep/Delay behaviour
ISR(WDT_vect) { Sleepy::watchdogEvent(); } // Setup the watchdog

const bool prefer_sleep = true;		// Specifies whether to use delay (false) or sleep (true) for long delays without sensor power
const bool high_power = true;		// Specifies whether to use delay (true) or sleep (false) for periods when power supplied to sensors
const bool shutdown = true;		// Specifies whether to shutdown power to the components (true) during the time between measurements or not (false)
const bool block_serial = true;	// Specifies whether to send debugging info through the serial port (false)

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
//const unsigned long SENSOR_DELAY = 10000;	// The sleep period (ms) - DEBUGGING
const unsigned long DELAY_PERIODS = 10;		// The number of sleep periods before updating sensor readings
const unsigned long STARTUP_DELAY = 3000;	// The period allowed for component warmup and initialization (ms).
const unsigned long COMM_DELAY = 1000;		// The period allowed for XBee communications to initialize/finalize (ms).

// Local pins
const int POWER_PIN = 12;	// The digital pin for powering the sensors and XBee
const int DS18S20_PIN = 2;	// The digital pin where the temperature probe is connected

// Sensor objects
HTU21D airSensor;
MAX1704 batterySensor;
AutoLightSensor lightSensor;
OneWire tempSensor(DS18S20_PIN);

// Union for conversion of numbers to byte arrays
union FloatConverter {
	float f;
	uint8_t b[sizeof(float)];
};

void SmartDelay(unsigned long delay_time, bool force_delay);
void Message(String msg);
float getTemp();

//=============================================================================
// SETUP
//=============================================================================
void setup() {
	// Initialize the pins
	pinMode(POWER_PIN, OUTPUT);		// Set the power pin to digital output

	// Start the I2C interface
	lightSensor.begin();
	
	// Setup the serial communications
	Serial.begin(9600);
	Message("Starting Serial...");
	delay(COMM_DELAY);

	// Start the power monitor
	Message("Setting up battery monitor");
	batterySensor.reset();		// Do a power reset
	batterySensor.quickStart();	// Quickly assess the state of charge (SOC)

	// Start the XBee connection, if not in shutdown mode
	if(!shutdown) {
		// Connect to the XBee
		Message("Starting XBee connection...");
		xbeeSerial.begin(9600);
		localRadio.setSerial(xbeeSerial);
		Message("FINISHED");
	}

	// Shutdown sensor power setup
	if(shutdown) digitalWrite(POWER_PIN, LOW);	// Turn off the power
	else digitalWrite(POWER_PIN, HIGH);			// Turn on the power
}

//=============================================================================
// MAIN LOOP
//=============================================================================
void loop() {
	//-------------------------------------------------------------------------
	// POWER UP COMPONENTS
	//-------------------------------------------------------------------------
	if(shutdown) {
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
	}

	//-------------------------------------------------------------------------
	// COLLECT SENSOR DATA
	//-------------------------------------------------------------------------
	// Read the luminosity
	Message("Reading the luminosity");
	FloatConverter Luminosity;
	Luminosity.f = lightSensor.getLuminosity();

	// Read the temperature from the external sensor
	Message("Reading tempeature");
	FloatConverter Temperature;
	Temperature.f = getTemp();

	// Read the humidity
	Message("Reading humidity");
	FloatConverter Humidity;
	float raw_humidity = airSensor.readHumidity();
	float raw_temperature = airSensor.readTemperature();
	Humidity.f = raw_humidity - 0.15*(25.0 - raw_temperature);	// Correction for HTU21D from spec sheet
	// Humidity.f = raw_humidity;

	// Read the battery voltage
	Message("Reading battery voltage");
	FloatConverter Power;
	Power.f = batterySensor.cellVoltage();

	// Read the battery SOC
	Message("Reading battery state of charge");
	FloatConverter SOC;
	SOC.f = batterySensor.stateOfCharge();

	// Sleep the battery sensor
	if(shutdown) {
		Message("Putting battery sensor to sleep");
		batterySensor.sleep();
	}

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
/*	String xbee_message = "Sent the following message(";
	xbee_message += String((double)Temperature.f, 2);
	xbee_message += "," + String(Humidity.f);
	xbee_message += "," + String(Power.f);
	xbee_message += "," + String(SOC.f);
	xbee_message += "," + String(Luminosity.f);
	xbee_message += "): ";
	for(int i = 0; i < sizeof(package); i++) {
		if(i != 0) xbee_message += "-";
		xbee_message += String(package[i], HEX);
	}
	Message(xbee_message);*/

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
	if(shutdown) {
		Message("Turn off power to components");
		xbeeSerial.end();	// Turn off the serial communication with the xbee
		digitalWrite(POWER_PIN, LOW);
	}

	// Cycle delay
	Message("Sensor delay");
	for(int i = 0; i < DELAY_PERIODS; i++) {
		String loop_msg = "Running delay " + String(i+1);
		Message(loop_msg);
		SmartDelay(SENSOR_DELAY, false);
	}
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
	if(!block_serial) {
		Serial.print(millis());
		Serial.print(": ");
		Serial.println(msg);
	}
}

//=============================================================================
// getTemp
//=============================================================================
float getTemp() {	// Returns the temperature from one DS18S20 in DEG Celsius
	byte data[12];	// Holds temperature data
	byte addr[8];	// Holds address of the sensor

	if(!tempSensor.search(addr)) {
		//no more sensors on chain, reset search
		tempSensor.reset_search();
		return -1000;
	}

	if(OneWire::crc8(addr, 7) != addr[7]) {
		Message("CRC is not valid!");
		return -1000;
	}

	if(addr[0] != 0x10 && addr[0] != 0x28) {
		Message("Device is not recognized");
		return -1000;
	}

	tempSensor.reset();
	tempSensor.select(addr);
	tempSensor.write(0x44, 1);	// Start conversion, with parasite power on at the end

	delay(750);	// Wait for temperature conversion to complete

	byte present = tempSensor.reset();
	tempSensor.select(addr);
	tempSensor.write(0xBE);	// Read Scratchpad


	for(int i = 0; i < 9; i++) {	// We need 9 bytes
		data[i] = tempSensor.read();
	}

	tempSensor.reset_search();

	byte MSB = data[1];
	byte LSB = data[0];

	float tempRead = ((MSB << 8) | LSB);	// Using two's compliment
	float TemperatureSum = tempRead/16;

	return TemperatureSum;
}
