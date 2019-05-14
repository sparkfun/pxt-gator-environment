/******************************************************************************
SparkFunBME280.cpp
BME280 Arduino and Teensy Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/BME280_Breakout

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE 1.8.5
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/
//See SparkFunBME280.h for additional topology notes.

#include "SparkFunEnvironment.h"
#include "mbed.h"
#include "MicroBit.h"

MicroBit uBit;

//****************************************************************************//
//
//  BMEsettings and configuration
//
//****************************************************************************//

//Constructor -- Specifies default configuration
environment::environment( void )
{
	//Construct with these default BMEsettings

	BMEsettings.commInterface = I2C_MODE; //Default to I2C

	BMEsettings.I2CAddress = 0x77; //Default, jumper open is 0x77

	BMEsettings.chipSelectPin = 10; //Select CS pin for SPI
	CCSsettings.I2CAddress = 0x5A;
	
	//These are deprecated BMEsettings
	BMEsettings.runMode = 3; //Normal/Run
	BMEsettings.tStandby = 0; //0.5ms
	BMEsettings.filter = 0; //Filter off
	BMEsettings.tempOverSample = 1;
	BMEsettings.pressOverSample = 1;
	BMEsettings.humidOverSample = 1;
    BMEsettings.tempCorrection = 0.0; // correction of temperature - added to the result
}


//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorBMEsettings to start the IMU
//  Use statements such as "mySensor.BMEsettings.commInterface = SPI_MODE;" to 
//  configure before calling .begin();
//
//****************************************************************************//
void environment::begin()
{
	uBit.sleep(2);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.

	BMEsettings.commInterface = I2C_MODE;
	//Check communication with IC before anything else

	//Reading all compensation data, range 0x88:A1, 0xE1:E7
	calibration.dig_T1 = ((uint16_t)((readRegister(BME280_ADDRESS, BME280_DIG_T1_MSB_REG) << 8) + readRegister(BME280_ADDRESS, BME280_DIG_T1_LSB_REG)));
	calibration.dig_T2 = ((int16_t)((readRegister(BME280_ADDRESS, BME280_DIG_T2_MSB_REG) << 8) + readRegister(BME280_ADDRESS, BME280_DIG_T2_LSB_REG)));
	calibration.dig_T3 = ((int16_t)((readRegister(BME280_ADDRESS, BME280_DIG_T3_MSB_REG) << 8) + readRegister(BME280_ADDRESS, BME280_DIG_T3_LSB_REG)));

	calibration.dig_P1 = ((uint16_t)((readRegister(BME280_ADDRESS, BME280_DIG_P1_MSB_REG) << 8) + readRegister(BME280_ADDRESS, BME280_DIG_P1_LSB_REG)));
	calibration.dig_P2 = ((int16_t)((readRegister(BME280_ADDRESS, BME280_DIG_P2_MSB_REG) << 8) + readRegister(BME280_ADDRESS, BME280_DIG_P2_LSB_REG)));
	calibration.dig_P3 = ((int16_t)((readRegister(BME280_ADDRESS, BME280_DIG_P3_MSB_REG) << 8) + readRegister(BME280_ADDRESS, BME280_DIG_P3_LSB_REG)));
	calibration.dig_P4 = ((int16_t)((readRegister(BME280_ADDRESS, BME280_DIG_P4_MSB_REG) << 8) + readRegister(BME280_ADDRESS, BME280_DIG_P4_LSB_REG)));
	calibration.dig_P5 = ((int16_t)((readRegister(BME280_ADDRESS, BME280_DIG_P5_MSB_REG) << 8) + readRegister(BME280_ADDRESS, BME280_DIG_P5_LSB_REG)));
	calibration.dig_P6 = ((int16_t)((readRegister(BME280_ADDRESS, BME280_DIG_P6_MSB_REG) << 8) + readRegister(BME280_ADDRESS, BME280_DIG_P6_LSB_REG)));
	calibration.dig_P7 = ((int16_t)((readRegister(BME280_ADDRESS, BME280_DIG_P7_MSB_REG) << 8) + readRegister(BME280_ADDRESS, BME280_DIG_P7_LSB_REG)));
	calibration.dig_P8 = ((int16_t)((readRegister(BME280_ADDRESS, BME280_DIG_P8_MSB_REG) << 8) + readRegister(BME280_ADDRESS, BME280_DIG_P8_LSB_REG)));
	calibration.dig_P9 = ((int16_t)((readRegister(BME280_ADDRESS, BME280_DIG_P9_MSB_REG) << 8) + readRegister(BME280_ADDRESS, BME280_DIG_P9_LSB_REG)));

	calibration.dig_H1 = ((uint8_t)(readRegister(BME280_ADDRESS, BME280_DIG_H1_REG)));
	calibration.dig_H2 = ((int16_t)((readRegister(BME280_ADDRESS, BME280_DIG_H2_MSB_REG) << 8) + readRegister(BME280_ADDRESS, BME280_DIG_H2_LSB_REG)));
	calibration.dig_H3 = ((uint8_t)(readRegister(BME280_ADDRESS, BME280_DIG_H3_REG)));
	calibration.dig_H4 = ((int16_t)((readRegister(BME280_ADDRESS, BME280_DIG_H4_MSB_REG) << 4) + (readRegister(BME280_ADDRESS, BME280_DIG_H4_LSB_REG) & 0x0F)));
	calibration.dig_H5 = ((int16_t)((readRegister(BME280_ADDRESS, BME280_DIG_H5_MSB_REG) << 4) + ((readRegister(BME280_ADDRESS, BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
	calibration.dig_H6 = ((int8_t)readRegister(BME280_ADDRESS, BME280_DIG_H6_REG));

	//Most of the time the sensor will be init with default values
	//But in case user has old/deprecated code, use the BMEsettings.x values
	setStandbyTime(BMEsettings.tStandby);
	setFilter(BMEsettings.filter);
	setPressureOverSample(BMEsettings.pressOverSample); //Default of 1x oversample
	setHumidityOverSample(BMEsettings.humidOverSample); //Default of 1x oversample
	setTempOverSample(BMEsettings.tempOverSample); //Default of 1x oversample
	
	setMode(MODE_NORMAL); //Go!
	
	uint8_t data[4] = {0x11,0xE5,0x72,0x8A}; //Reset key

		//Reset the device
	multiWriteRegister(CCS811_ADDRESS, CCS811_SW_RESET, data, 4);

	//Tclk = 1/16MHz = 0x0000000625
	//0.001 s / tclk = 16000 counts
	volatile uint8_t temp = 0;

	for( uint32_t i = 0; i < 200000; i++ ) //Spin for a good while
	{
		temp++;
	}
	
	//Write 0 bytes to this register to start app
	uBit.i2c.write(CCS811_ADDRESS, (char *)CCS811_APP_START, 1);
	
	//Added from issue 6
	// Without a delay here, the CCS811 and I2C can be put in a bad state.
	// Seems to work with 50us delay, but make a bit longer to be sure.
	
	setDriveMode(1); //Read every second	
}

bool environment::checkForStatusError( void )
{
	uint8_t value = readRegister(CCS811_ADDRESS, CCS811_STATUS);
	return (value & 1 << 0);
}

//Checks to see if APP_VALID flag is set in the status register
bool environment::appValid( void )
{
	uint8_t value = readRegister(CCS811_ADDRESS, CCS811_STATUS);
	return (value & 1 << 4);
}

//Set the mode bits in the ctrl_meas register
// Mode 00 = Sleep
// 01 and 10 = Forced
// 11 = Normal mode
void environment::setMode(uint8_t mode)
{
	if(mode > 0b11) mode = 0; //Error check. Default to sleep mode
	
	uint8_t controlData = readRegister(BME280_ADDRESS, BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<1) | (1<<0) ); //Clear the mode[1:0] bits
	controlData |= mode; //Set
	writeRegister(BME280_ADDRESS, BME280_CTRL_MEAS_REG, controlData);
}

//Mode 0 = Idle
//Mode 1 = read every 1s
//Mode 2 = every 10s
//Mode 3 = every 60s
//Mode 4 = RAW mode
void environment::setDriveMode( uint8_t mode )
{
	if (mode > 4) mode = 4; //sanitize input

	uint8_t value = readRegister(CCS811_ADDRESS, CCS811_MEAS_MODE); //Read what's currently there
	value &= ~(0b00000111 << 4); //Clear DRIVE_MODE bits
	value |= (mode << 4); //Mask in mode
	writeRegister(CCS811_ADDRESS, CCS811_MEAS_MODE, value);

}

//****************************************************************************//
//
//  Sensor functions
//
//****************************************************************************//
//Updates the total voltatile organic compounds (TVOC) in parts per billion (PPB)
//and the CO2 value
//Returns nothing
void environment::readAlgorithmResults( void )
{
	uint8_t data[4];
	readRegisterRegion(CCS811_ADDRESS, data, CCS811_ALG_RESULT_DATA, 4);
	// Data ordered:
	// co2MSB, co2LSB, tvocMSB, tvocLSB

	CO2 = ((uint16_t)data[0] << 8) | data[1];
	tVOC = ((uint16_t)data[2] << 8) | data[3];
}

uint16_t environment::getTVOC( void )
{
	return tVOC;
}

uint16_t environment::getCO2( void )
{
	return CO2;
}

//Gets the current mode bits in the ctrl_meas register
//Mode 00 = Sleep
// 01 and 10 = Forced
// 11 = Normal mode
uint8_t environment::getMode()
{
	uint8_t controlData = readRegister(BME280_ADDRESS, BME280_CTRL_MEAS_REG);
	return(controlData & 0b00000011); //Clear bits 7 through 2
}

//Set the standby bits in the config register
//tStandby can be:
//  0, 0.5ms
//  1, 62.5ms
//  2, 125ms
//  3, 250ms
//  4, 500ms
//  5, 1000ms
//  6, 10ms
//  7, 20ms
void environment::setStandbyTime(uint8_t timeSetting)
{
	if(timeSetting > 0b111) timeSetting = 0; //Error check. Default to 0.5ms
	
	uint8_t controlData = readRegister(BME280_ADDRESS, BME280_CONFIG_REG);
	controlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear the 7/6/5 bits
	controlData |= (timeSetting << 5); //Align with bits 7/6/5
	writeRegister(BME280_ADDRESS, BME280_CONFIG_REG, controlData);
}

//Set the filter bits in the config register
//filter can be off or number of FIR coefficients to use:
//  0, filter off
//  1, coefficients = 2
//  2, coefficients = 4
//  3, coefficients = 8
//  4, coefficients = 16
void environment::setFilter(uint8_t filterSetting)
{
	if(filterSetting > 0b111) filterSetting = 0; //Error check. Default to filter off
	
	uint8_t controlData = readRegister(BME280_ADDRESS, BME280_CONFIG_REG);
	controlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear the 4/3/2 bits
	controlData |= (filterSetting << 2); //Align with bits 4/3/2
	writeRegister(BME280_ADDRESS, BME280_CONFIG_REG, controlData);
}

//Set the temperature oversample value
//0 turns off temp sensing
//1 to 16 are valid over sampling values
void environment::setTempOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_t bits (7, 6, 5) to overSampleAmount
	uint8_t controlData = readRegister(BME280_ADDRESS, BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear bits 765
	controlData |= overSampleAmount << 5; //Align overSampleAmount to bits 7/6/5
	writeRegister(BME280_ADDRESS, BME280_CTRL_MEAS_REG, controlData);
	
	setMode(originalMode); //Return to the original user's choice
}

//Set the pressure oversample value
//0 turns off pressure sensing
//1 to 16 are valid over sampling values
void environment::setPressureOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_p bits (4, 3, 2) to overSampleAmount
	uint8_t controlData = readRegister(BME280_ADDRESS, BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear bits 432
	controlData |= overSampleAmount << 2; //Align overSampleAmount to bits 4/3/2
	writeRegister(BME280_ADDRESS, BME280_CTRL_MEAS_REG, controlData);
	
	setMode(originalMode); //Return to the original user's choice
}

//Set the humidity oversample value
//0 turns off humidity sensing
//1 to 16 are valid over sampling values
void environment::setHumidityOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_h bits (2, 1, 0) to overSampleAmount
	uint8_t controlData = readRegister(BME280_ADDRESS, BME280_CTRL_HUMIDITY_REG);
	controlData &= ~( (1<<2) | (1<<1) | (1<<0) ); //Clear bits 2/1/0
	controlData |= overSampleAmount << 0; //Align overSampleAmount to bits 2/1/0
	writeRegister(BME280_ADDRESS, BME280_CTRL_HUMIDITY_REG, controlData);

	setMode(originalMode); //Return to the original user's choice
}

//Validates an over sample value
//Allowed values are 0 to 16
//These are used in the humidty, pressure, and temp oversample functions
uint8_t environment::checkSampleValue(uint8_t userValue)
{
	switch(userValue) 
	{
		case(0): 
			return 0;
			break; //Valid
		case(1): 
			return 1;
			break; //Valid
		case(2): 
			return 2;
			break; //Valid
		case(4): 
			return 3;
			break; //Valid
		case(8): 
			return 4;
			break; //Valid
		case(16): 
			return 5;
			break; //Valid
		default: 
			return 1; //Default to 1x
			break; //Good
	}
}

//Set the global setting for the I2C address we want to communicate with
//Default is 0x77
void environment::setI2CAddress(uint8_t address)
{
	BMEsettings.I2CAddress = address; //Set the I2C address for this device
}

//Check the measuring bit and return true while device is taking measurement
bool environment::isMeasuring(void)
{
	uint8_t stat = readRegister(BME280_ADDRESS, BME280_STAT_REG);
	return(stat & (1<<3)); //If the measuring bit (3) is set, return true
}

//Strictly resets.  Run .begin() afterwards
void environment::reset( void )
{
	writeRegister(BME280_ADDRESS, BME280_RST_REG, 0xB6);
	
}

//****************************************************************************//
//
//  Pressure Section
//
//****************************************************************************//
float environment::readFloatPressure( void )
{

	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
    uint8_t buffer[3];
	readRegisterRegion(BME280_ADDRESS, buffer, BME280_PRESSURE_MSB_REG, 3);
    int32_t adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
	
	int64_t var1, var2, p_acc;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calibration.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calibration.dig_P5)<<17);
	var2 = var2 + (((int64_t)calibration.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)calibration.dig_P3)>>8) + ((var1 * (int64_t)calibration.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration.dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p_acc = 1048576 - adc_P;
	p_acc = (((p_acc<<31) - var2)*3125)/var1;
	var1 = (((int64_t)calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
	var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
	p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7)<<4);
	
	return (float)p_acc / 256.0;
	
}

//Sets the internal variable _referencePressure so the 
void environment::setReferencePressure(float refPressure)
{
	_referencePressure = refPressure;
}

//Return the local reference pressure
float environment::getReferencePressure()
{
	return(_referencePressure);
}

float environment::readFloatAltitudeMeters( void )
{
	float heightOutput = 0;
	
	//heightOutput = ((float)-45846.2)*(pow(((float)readFloatPressure()/(float)_referencePressure), 0.190263) - (float)1);
	heightOutput = ((float)-44330.77)*(pow(((float)readFloatPressure()/(float)_referencePressure), 0.190263) - (float)1); //Corrected, see issue 30
	return heightOutput;
	
}

float environment::readFloatAltitudeFeet( void )
{
	float heightOutput = 0;
	
	heightOutput = readFloatAltitudeMeters() * 3.28084;
	return heightOutput;
	
}

//****************************************************************************//
//
//  Humidity Section
//
//****************************************************************************//
float environment::readFloatHumidity( void )
{
	
	// Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
    uint8_t buffer[2];
	readRegisterRegion(BME280_ADDRESS, buffer, BME280_HUMIDITY_MSB_REG, 2);
    int32_t adc_H = ((uint32_t)buffer[0] << 8) | ((uint32_t)buffer[1]);
	
	int32_t var1;
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) * var1)) +
	((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) * (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)calibration.dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	return (float)(var1>>12) / 1024.0;
}

//****************************************************************************//
//
//  Temperature Section
//
//****************************************************************************//

float environment::readTempC( void )
{
	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
	// t_fine carries fine temperature as global value

	//get the reading (adc_T);
    uint8_t buffer[3];
	readRegisterRegion(BME280_ADDRESS, buffer, BME280_TEMPERATURE_MSB_REG, 3);
    int32_t adc_T = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);

	//By datasheet, calibrate
	int64_t var1, var2;

	var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) *
	((int32_t)calibration.dig_T3)) >> 14;
	t_fine = var1 + var2;
	float output = (t_fine * 5 + 128) >> 8;

	output = output / 100 + BMEsettings.tempCorrection;
	
	return output;
}

float environment::readTempF( void )
{
	float output = readTempC();
	output = (output * 9) / 5 + 32;

	return output;
}

//****************************************************************************//
//
//  Dew point Section
//
//****************************************************************************//
// Returns Dew point in DegC
double environment::dewPointC(void)
{
  double celsius = readTempC(); 
  double humidity = readFloatHumidity();
  // (1) Saturation Vapor Pressure = ESGG(T)
  double RATIO = 373.15 / (273.15 + celsius);
  double RHS = -7.90298 * (RATIO - 1);
  RHS += 5.02808 * log10(RATIO);
  RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
  RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
  RHS += log10(1013.246);
         // factor -3 is to adjust units - Vapor Pressure SVP * humidity
  double VP = pow(10, RHS - 3) * humidity;
         // (2) DEWPOINT = F(Vapor Pressure)
  double T = log(VP/0.61078);   // temp var
  return (241.88 * T) / (17.558 - T);
}

// Returns Dew point in DegF
double environment::dewPointF(void)
{
	return(dewPointC() * 1.8 + 32); //Convert C to F
}

//****************************************************************************//
//
//  Utility
//
//****************************************************************************//
void environment::readRegisterRegion(uint8_t address, uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	uBit.i2c.readRegister(address, offset, outputPointer, length);	
}

uint8_t environment::readRegister(uint8_t address, uint8_t offset)
{
	return uBit.i2c.readRegister(address, offset);
}

int16_t environment::readRegisterInt16(uint8_t address, uint8_t offset )
{
	uint8_t myBuffer[2];
	
	readRegisterRegion(address, myBuffer, offset, 2);  //Does memory transfer
	int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);
	
	return output;
}

void environment::writeRegister(uint8_t address, uint8_t offset, uint8_t dataToWrite)
{
	uBit.i2c.writeRegister(address, offset, dataToWrite);
}

void environment::multiWriteRegister(uint8_t address, uint8_t offset, uint8_t *inputPointer, uint8_t length)
{
	uint8_t realLength = length + 1;
	char temp[realLength];
	temp[0] = offset;
	memcpy(&temp[1], inputPointer, length); //tempLong is 4 bytes, we only need 3
	uBit.i2c.write(address, temp, realLength);
}