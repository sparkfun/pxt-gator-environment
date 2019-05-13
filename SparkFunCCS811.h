/******************************************************************************
SparkFunCCS811.h
CCS811 Arduino library

Marshall Taylor @ SparkFun Electronics
Nathan Seidle @ SparkFun Electronics

April 4, 2017

https://github.com/sparkfun/CCS811_Air_Quality_Breakout
https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library

Resources:
Uses Wire.h for i2c operation

Development environment specifics:
Arduino IDE 1.8.1

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __CCS811_H__
#define __CCS811_H__

#include "stdint.h"

#include "mbed.h"
#include "MicroBit.h"

//Register addresses
#define CCS811_STATUS 0x00
#define CCS811_MEAS_MODE 0x01
#define CCS811_ALG_RESULT_DATA 0x02
#define CCS811_RAW_DATA 0x03
#define CCS811_ENV_DATA 0x05
#define CCS811_NTC 0x06
#define CCS811_THRESHOLDS 0x10
#define CCS811_BASELINE 0x11
#define CCS811_HW_ID 0x20
#define CCS811_HW_VERSION 0x21
#define CCS811_FW_BOOT_VERSION 0x23
#define CCS811_FW_APP_VERSION 0x24
#define CCS811_ERROR_ID 0xE0
#define CCS811_APP_START 0xF4
#define CCS811_SW_RESET 0xFF

//This is the core operational class of the driver.
//  CCS811Core contains only read and write operations towards the sensor.
//  To use the higher level functions, use the class CCS811 which inherits
//  this class.



//This is the highest level class of the driver.
//
//  class CCS811 inherits the CCS811Core and makes use of the beginCore()
//method through its own begin() method.  It also contains user settings/values.

class CCS811
{
public:
	CCS811( void );

	//Call to check for errors, start app, and set default mode 1
	bool begin(MicroBitI2C &wirePort); //Use the Wire hardware by default
	bool beginCore(MicroBitI2C &wirePort);
	
	void readAlgorithmResults( void );
	bool checkForStatusError( void );
	bool dataAvailable( void );
	bool appValid( void );
	uint8_t getErrorRegister( void );
	uint16_t getBaseline( void );
	void setBaseline( uint16_t );
	void enableInterrupts( void );
	void disableInterrupts( void );
	void setDriveMode( uint8_t mode );
	bool setEnvironmentalData( float relativeHumidity, float temperature );
	void setRefResistance( float );
	void readNTC( void );
	uint16_t getTVOC( void );
	uint16_t getCO2( void );
	float getResistance( void );
	float getTemperature( void );
	//readRegister reads one 8-bit register
	void readRegister( uint8_t offset, uint8_t* outputPointer);
	//multiReadRegister takes a uint8 array address as input and performs
	//  a number of consecutive reads
	void multiReadRegister(uint8_t offset, uint8_t *outputPointer, uint8_t length);

	//***Writing functions***//
	
	//Writes an 8-bit byte;
	void writeRegister(uint8_t offset, uint8_t dataToWrite);
	//multiWriteRegister takes a uint8 array address as input and performs
	//  a number of consecutive writes
	void multiWriteRegister(uint8_t offset, uint8_t *inputPointer, uint8_t length);
	
private:
	//These are the air quality values obtained from the sensor
    MicroBitI2C *_i2cPort; //The generic connection to user's chosen I2C hardware
	uint8_t I2CAddress;
	float refResistance;
	float resistance;
	uint16_t tVOC = 0;
	uint16_t CO2 = 0;
	uint16_t vrefCounts = 0;
	uint16_t ntcCounts = 0;
	float temperature = 0;	
};

#endif  // End of definition check
