/******************************************************************************
SparkFunBME280.h
BME280 Arduino and Teensy Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/BME280_Breakout

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.

TODO:
	roll library ver to 2.0
	remove hard wire.
	write escalating examples


******************************************************************************/

#ifndef __MBED_H_
#define __MBED_H_
#include "mbed.h"
#endif

#define I2C_MODE 0
#define SPI_MODE 1

#define NO_WIRE 0
#define HARD_WIRE 1
#define SOFT_WIRE 2

//This is the main operational class of the driver.

class environment
{
  public:
    //settings
	
	//Constructor generates default SensorSettings.
	//(over-ride after construction if desired)
    environment( void );
    //~BME280() = default;
	
	//Call to apply SensorSettings.
	//This also gets the SensorCalibration constants
    void begin( void );

	uint8_t getMode(void); //Get the current mode: sleep, forced, or normal
	void setMode(uint8_t mode); //Set the current mode

	void setTempOverSample(uint8_t overSampleAmount); //Set the temperature sample mode
	void setPressureOverSample(uint8_t overSampleAmount); //Set the pressure sample mode
	void setHumidityOverSample(uint8_t overSampleAmount); //Set the humidity sample mode
	void setStandbyTime(uint8_t timeSetting); //Set the standby time between measurements
	void setFilter(uint8_t filterSetting); //Set the filter

	void setReferencePressure(float refPressure); //Allows user to set local sea level reference pressure
	float getReferencePressure();
	void readAlgorithmResults(void);
	bool dataAvailable( void );
	
	uint16_t getTVOC( void );
	uint16_t getCO2( void );
	
	bool isMeasuring(void); //Returns true while the device is taking measurement
	
	//Software reset routine
	void reset( void );
	
    //Returns the values as floats.
    float readFloatPressure( void );
	float readFloatAltitudeMeters( void );
	float readFloatAltitudeFeet( void );
	
	float readFloatHumidity( void );

	//Temperature related methods
    float readTempC( void );
    float readTempF( void );

	//Dewpoint related methods
	//From Pavel-Sayekat: https://github.com/sparkfun/SparkFun_BME280_Breakout_Board/pull/6/files
    double dewPointC(void);
    double dewPointF(void);
	
	bool checkForStatusError( void );
	bool appValid( void );
	void setDriveMode( uint8_t mode );
    //The following utilities read and write

	//ReadRegisterRegion takes a uint8 array address as input and reads
	//a chunk of memory into that array.
    void readRegisterRegion(uint8_t address, uint8_t *outputPointer , uint8_t offset, uint8_t length);
	//readRegister reads one register
    uint8_t readRegister(uint8_t, uint8_t);
    //Reads two regs, LSByte then MSByte order, and concatenates them
	//Used for two-byte reads
	int16_t readRegisterInt16(uint8_t, uint8_t offset );
	//Writes a byte;
    void writeRegister(uint8_t, uint8_t, uint8_t);

	void multiWriteRegister(uint8_t address, uint8_t offset, uint8_t *inputPointer, uint8_t length);
private:
	uint8_t checkSampleValue(uint8_t userValue); //Checks for valid over sample values

    uint8_t _wireType = HARD_WIRE; //Default to Wire.h
	
  	float _referencePressure = 101325.0; //Default but is changeable
};
