/**
 * James Luther and Michael Schneider @ University of Colorado Boulder
 * 
 * Environment Class
 */

const BME280_ADDRESS = 0x77;
const BME280_DIG_T1_LSB_REG = 0x88;
const BME280_DIG_T1_MSB_REG = 0x89;
const BME280_DIG_T2_LSB_REG = 0x8A;
const BME280_DIG_T2_MSB_REG = 0x8B;
const BME280_DIG_T3_LSB_REG = 0x8C;
const BME280_DIG_T3_MSB_REG = 0x8D;
const BME280_DIG_P1_LSB_REG = 0x8E;
const BME280_DIG_P1_MSB_REG = 0x8F;
const BME280_DIG_P2_LSB_REG = 0x90;
const BME280_DIG_P2_MSB_REG = 0x91;
const BME280_DIG_P3_LSB_REG = 0x92;
const BME280_DIG_P3_MSB_REG = 0x93;
const BME280_DIG_P4_LSB_REG = 0x94;
const BME280_DIG_P4_MSB_REG = 0x95;
const BME280_DIG_P5_LSB_REG = 0x96;
const BME280_DIG_P5_MSB_REG = 0x97;
const BME280_DIG_P6_LSB_REG = 0x98;
const BME280_DIG_P6_MSB_REG = 0x99;
const BME280_DIG_P7_LSB_REG = 0x9A;
const BME280_DIG_P7_MSB_REG = 0x9B;
const BME280_DIG_P8_LSB_REG = 0x9C;
const BME280_DIG_P8_MSB_REG = 0x9D;
const BME280_DIG_P9_LSB_REG = 0x9E;
const BME280_DIG_P9_MSB_REG = 0x9F;
const BME280_DIG_H1_REG = 0xA1;
const BME280_CHIP_ID_REG = 0xD0; //Chip ID
const BME280_RST_REG = 0xE0; //Softreset Reg
const BME280_DIG_H2_LSB_REG = 0xE1;
const BME280_DIG_H2_MSB_REG = 0xE2;
const BME280_DIG_H3_REG = 0xE3;
const BME280_DIG_H4_MSB_REG = 0xE4;
const BME280_DIG_H4_LSB_REG = 0xE5;
const BME280_DIG_H5_MSB_REG = 0xE6;
const BME280_DIG_H6_REG = 0xE7;
const BME280_CTRL_HUMIDITY_REG = 0xF2; //Ctrl Humidity Reg
const BME280_STAT_REG = 0xF3; //Status Reg
const BME280_CTRL_MEAS_REG = 0xF4; //Ctrl Measure Reg
const BME280_CONFIG_REG = 0xF5; //Configuration Reg
const BME280_PRESSURE_MSB_REG = 0xF7; //Pressure MSB
const BME280_PRESSURE_LSB_REG = 0xF8; //Pressure LSB
const BME280_PRESSURE_XLSB_REG = 0xF9; //Pressure XLSB
const BME280_TEMPERATURE_MSB_REG = 0xFA; //Temperature MSB
const BME280_TEMPERATURE_LSB_REG = 0xFB; //Temperature LSB
const BME280_TEMPERATURE_XLSB_REG = 0xFC; //Temperature XLSB
const BME280_HUMIDITY_MSB_REG = 0xFD; //Humidity MSB
const BME280_HUMIDITY_LSB_REG = 0xFE; //Humidity LSB

const CCS811_ADDRESS = 0x5B;
const CCS811_STATUS = 0x00;
const CCS811_MEAS_MODE = 0x01;
const CCS811_ALG_RESULT_DATA = 0x02;
const CCS811_RAW_DATA = 0x03;
const CCS811_ENV_DATA = 0x05;
const CCS811_NTC = 0x06;
const CCS811_THRESHOLDS = 0x10;
const CCS811_BASELINE = 0x11;
const CCS811_HW_ID = 0x20;
const CCS811_HW_VERSION = 0x21;
const CCS811_FW_BOOT_VERSION = 0x23;
const CCS811_FW_APP_VERSION = 0x24;
const CCS811_ERROR_ID = 0xE0;
const CCS811_APP_START = 0xF4;
const CCS811_SW_RESET = 0xFF;


const MODE_SLEEP = 0b00;
const MODE_FORCED = 0b01;
const MODE_NORMAL = 0b11;


class Environment {

    tVOC: number = 0;
    CO2: number = 0;
    temperature: number = 0;
    pressure: number = 0;
    humidity: number = 0;

    t_fine: number;

    calibration = new SensorCalibration();
    BMErunMode: number;
    BMEtStandby: number;
    BMEfilter: number;
    BMEtempOverSample: number;
    BMEpressOverSample: number;
    BMEhumidOverSample: number;
    BMEtempCorrection: number;

    _referencePressure = 101325.0; //Default but is changeable


    constructor(){
        return;
    }

    //Call to apply SensorSettings.
    //This also gets the SensorCalibration constants
    begin(){

        //Reset the device
        let data: number[]= [0x11, 0xE5, 0x72, 0x8A]; //Reset key
        this.writeRegisterShort(CCS811_ADDRESS, CCS811_SW_RESET);

        for (let i=0; i < 4; i++){
            this.writeRegisterShort(CCS811_ADDRESS, data[i]);
        }

        //Tclk = 1/16MHz = 0x0000000625
        //0.001 s / tclk = 16000 counts
        let temp: number = 0;

        for(let i = 0; i < 200000; i++ ) //Spin for a good while
        {
            temp++;
        }
        
        // Check Status
        this.readRegister(CCS811_ADDRESS, CCS811_STATUS)
        
        //Write 0 bytes to this register to start app

        this.writeRegisterShort(CCS811_ADDRESS, CCS811_APP_START);

        this.setDriveMode(1);

        this.reset();

        // need 4 seconds before CCS811 can pull results properly
        this.BMErunMode = 3; //Normal/Run
        this.BMEtStandby = 0; //0.5ms
        this.BMEfilter = 0; //Filter off
        this.BMEtempOverSample = 1;
        this.BMEpressOverSample = 1;
        this.BMEhumidOverSample = 1;
        this.BMEtempCorrection = 0.0; // correction of temperature - added to the result
        //Check communication with IC before anything else

        //Reading all compensation data, range 0x88:A1, 0xE1:E7
        this.calibration.dig_T1 = (this.readRegisterUInt8LE(BME280_ADDRESS, BME280_DIG_T1_MSB_REG) << 8) + this.readRegisterUInt8LE(BME280_ADDRESS, BME280_DIG_T1_LSB_REG);
        this.calibration.dig_T2 = (this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_T2_MSB_REG) << 8) + this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_T2_LSB_REG);
        this.calibration.dig_T3 = (this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_T3_MSB_REG) << 8) + this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_T3_LSB_REG);

        this.calibration.dig_P1 = (this.readRegisterUInt8LE(BME280_ADDRESS, BME280_DIG_P1_MSB_REG) << 8) + this.readRegisterUInt8LE(BME280_ADDRESS, BME280_DIG_P1_LSB_REG);
        this.calibration.dig_P2 = (this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P2_MSB_REG) << 8) + this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P2_LSB_REG);
        this.calibration.dig_P3 = (this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P3_MSB_REG) << 8) + this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P3_LSB_REG);
        this.calibration.dig_P4 = (this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P4_MSB_REG) << 8) + this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P4_LSB_REG);
        this.calibration.dig_P5 = (this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P5_MSB_REG) << 8) + this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P5_LSB_REG);
        this.calibration.dig_P6 = (this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P6_MSB_REG) << 8) + this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P6_LSB_REG);
        this.calibration.dig_P7 = (this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P7_MSB_REG) << 8) + this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P7_LSB_REG);
        this.calibration.dig_P8 = (this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P8_MSB_REG) << 8) + this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P8_LSB_REG);
        this.calibration.dig_P9 = (this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P9_MSB_REG) << 8) + this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_P9_LSB_REG);

        this.calibration.dig_H1 = this.readRegisterUInt8LE(BME280_ADDRESS, BME280_DIG_H1_REG);
        this.calibration.dig_H2 = (this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_H2_MSB_REG) << 8) + this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_H2_LSB_REG);
        this.calibration.dig_H3 = this.readRegisterUInt8LE(BME280_ADDRESS, BME280_DIG_H3_REG);
        this.calibration.dig_H4 = (this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_H4_MSB_REG) << 4) + (this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_H4_LSB_REG) & 0x0F);
        this.calibration.dig_H5 = (this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_H5_MSB_REG) << 4) + ((this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_H4_LSB_REG) >> 4) & 0x0F);
        this.calibration.dig_H6 = this.readRegisterInt8LE(BME280_ADDRESS, BME280_DIG_H6_REG);

        //Most of the time the sensor will be init with default values
        //But in case user has old/deprecated code, use the BMEsettings.x values
        this.setStandbyTime(this.BMEtStandby);
        this.setFilter(this.BMEfilter);
        this.setPressureOverSample(this.BMEtempOverSample); //Default of 1x oversample
        this.setHumidityOverSample(this.BMEpressOverSample); //Default of 1x oversample
        this.setTempOverSample(this.BMEhumidOverSample); //Default of 1x oversample
        
        this.setMode(this.BMErunMode); //Go!

        pause(2000)

    };

    readAlgorithmResults(){

        pins.i2cWriteNumber(CCS811_ADDRESS, CCS811_ALG_RESULT_DATA, NumberFormat.UInt8LE, false)
        this.CO2 = pins.i2cReadNumber(CCS811_ADDRESS, NumberFormat.UInt16BE, true)
        this.tVOC = pins.i2cReadNumber(CCS811_ADDRESS, NumberFormat.UInt16BE, false)


    }
    dataAvailable(){
        let value: number = this.readRegister(CCS811_ADDRESS, CCS811_STATUS);
        return (value & 1 << 3);
    }

    getTVOC(){
        return this.tVOC;
    }

    getCO2(){
        return this.CO2;
    }

    //Mode 0 = Idle
    //Mode 1 = read every 1s
    //Mode 2 = every 10s
    //Mode 3 = every 60s
    //Mode 4 = RAW mode
    setDriveMode(mode: number) {
        if (mode > 4) {
            mode = 4; //sanitize input
        }
        let value = this.readRegister(CCS811_ADDRESS, CCS811_MEAS_MODE); //Read what's currently there
        value &= ~(0b00000111 << 4); //Clear DRIVE_MODE bits
        value |= (mode << 4); //Mask in mode
        this.writeRegister(CCS811_ADDRESS, CCS811_MEAS_MODE, value);

    }

    //Set the mode bits in the ctrl_meas register
    // Mode 00 = Sleep
    // 01 and 10 = Forced
    // 11 = Normal mode
    getMode(){
        let controlData: number = this.readRegister(BME280_ADDRESS, BME280_CTRL_MEAS_REG);
        return(controlData & 0b00000011); //Clear bits 7 through 2  

    } //Get the current mode: sleep, forced, or normal

    //Set the mode bits in the ctrl_meas register
    // Mode 00 = Sleep
    // 01 and 10 = Forced
    // 11 = Normal mode
    setMode(mode: number){
        if (mode > 0b11) {
            mode = 0; //Error check. Default to sleep mode
        }
    
        let controlData = this.readRegister(BME280_ADDRESS, BME280_CTRL_MEAS_REG);
        controlData &= ~( (1<<1) | (1<<0) ); //Clear the mode[1:0] bits
        controlData |= mode; //Set
        this.writeRegister(BME280_ADDRESS, BME280_CTRL_MEAS_REG, controlData);

    } //Set the current mode

    

    //Set the temperature oversample value
    //0 turns off temp sensing
    //1 to 16 are valid over sampling values
    setTempOverSample(overSampleAmount: number){
        overSampleAmount = this.checkSampleValue(overSampleAmount); //Error check
    
        let originalMode: number = this.getMode(); //Get the current mode so we can go back to it at the end
        
        this.setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

        //Set the osrs_t bits (7, 6, 5) to overSampleAmount
        let controlData: number = this.readRegister(BME280_ADDRESS, BME280_CTRL_MEAS_REG);
        controlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear bits 765
        controlData |= overSampleAmount << 5; //Align overSampleAmount to bits 7/6/5
        this.writeRegister(BME280_ADDRESS, BME280_CTRL_MEAS_REG, controlData);
        
        this.setMode(originalMode); //Return to the original user's choice

    } //Set the temperature sample mode

    //Set the pressure oversample value
    //0 turns off pressure sensing
    //1 to 16 are valid over sampling values
    setPressureOverSample(overSampleAmount: number){
        overSampleAmount = this.checkSampleValue(overSampleAmount); //Error check
    
        let originalMode: number = this.getMode(); //Get the current mode so we can go back to it at the end
        
        this.setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

        //Set the osrs_p bits (4, 3, 2) to overSampleAmount
        let controlData: number = this.readRegister(BME280_ADDRESS, BME280_CTRL_MEAS_REG);
        controlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear bits 432
        controlData |= overSampleAmount << 2; //Align overSampleAmount to bits 4/3/2
        this.writeRegister(BME280_ADDRESS, BME280_CTRL_MEAS_REG, controlData);
        
        this.setMode(originalMode); //Return to the original user's choice

    } //Set the pressure sample mode

    //Set the humidity oversample value
    //0 turns off humidity sensing
    //1 to 16 are valid over sampling values
    setHumidityOverSample(overSampleAmount: number){
        overSampleAmount = this.checkSampleValue(overSampleAmount); //Error check
        
        let originalMode: number = this.getMode(); //Get the current mode so we can go back to it at the end
        
        this.setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

        //Set the osrs_h bits (2, 1, 0) to overSampleAmount
        let controlData: number = this.readRegister(BME280_ADDRESS, BME280_CTRL_HUMIDITY_REG);
        controlData &= ~( (1<<2) | (1<<1) | (1<<0) ); //Clear bits 2/1/0
        controlData |= overSampleAmount << 0; //Align overSampleAmount to bits 2/1/0
        this.writeRegister(BME280_ADDRESS, BME280_CTRL_HUMIDITY_REG, controlData);

        this.setMode(originalMode); //Return to the original user's choice

    } //Set the humidity sample mode

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
    setStandbyTime(timeSetting: number){
        if (timeSetting > 0b111) {
            timeSetting = 0; //Error check. Default to 0.5ms
        }
    
        let controlData: number = this.readRegister(BME280_ADDRESS, BME280_CONFIG_REG);
        controlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear the 7/6/5 bits
        controlData |= (timeSetting << 5); //Align with bits 7/6/5
        this.writeRegister(BME280_ADDRESS, BME280_CONFIG_REG, controlData);

    } //Set the standby time between measurements

    //Set the filter bits in the config register
    //filter can be off or number of FIR coefficients to use:
    //  0, filter off
    //  1, coefficients = 2
    //  2, coefficients = 4
    //  3, coefficients = 8
    //  4, coefficients = 16
    setFilter(filterSetting: number){
        if(filterSetting > 0b111){ 
            filterSetting = 0; //Error check. Default to filter off
        }
        let controlData: number = this.readRegister(BME280_ADDRESS, BME280_CONFIG_REG);
        controlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear the 4/3/2 bits
        controlData |= (filterSetting << 2); //Align with bits 4/3/2
        this.writeRegister(BME280_ADDRESS, BME280_CONFIG_REG, controlData);

    } //Set the filter

    setReferencePressure(refPressure: any){
        this._referencePressure = refPressure;

    } //Allows user to set local sea level reference pressure

    getReferencePressure(){
        return this._referencePressure;
    }

    
    //Check the measuring bit and return true while device is taking measurement
    isMeasuring(){
        let stat: number = this.readRegister(BME280_ADDRESS, BME280_STAT_REG);
        return (stat & (1<<3)); //If the measuring bit (3) is set, return true
    }//Returns true while the device is taking measurement

    //Software reset routine: strictly resets. Run .begin() afterwards
    reset(){
        this.writeRegister(BME280_ADDRESS, BME280_RST_REG, 0xB6);

    }

    //Returns the values as floats.
    readFloatPressure(){

        // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
        // Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
        let buffer: number[] = this.readRegisterRegion(BME280_ADDRESS, BME280_PRESSURE_MSB_REG, 3);
        let adc_P: number = (buffer[0] << 12) | (buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
		//serial.writeString("adc_P = " + adc_P + "\n");
        let var1: number = (this.t_fine) - 128000;
        let var2: number = var1 * var1 * this.calibration.dig_P6;
        var2 = var2 + ((var1 * this.calibration.dig_P5)<<17);
        var2 = var2 + ((this.calibration.dig_P4)<<35);
        var1 = ((var1 * var1 * this.calibration.dig_P3)>>8) + ((var1 * this.calibration.dig_P2)<<12);
        var1 = ((((1)<<47)+var1))*(this.calibration.dig_P1)>>33;
        if (var1 == 0)
        {
            return 0; // avoid exception caused by division by zero
        }
        let p_acc = 1048576 - adc_P;
        p_acc = (((p_acc<<31) - var2)*3125)/var1;
        var1 = ((this.calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
        var2 = ((this.calibration.dig_P8) * p_acc) >> 19;
        p_acc = ((p_acc + var1 + var2) >> 8) + ((this.calibration.dig_P7)<<4);
        
        let pressure: number = p_acc / 256.0;
        return pressure;
    }
    readFloatAltitudeMeters(){
        let heightOutput: number = (-44330.77) * (Math.pow((this.readFloatPressure() / this._referencePressure), 0.190263) - 1); //Corrected, see issue 30
        return heightOutput;
    }
    readFloatAltitudeFeet(){
        let heightOutput: number = this.readFloatAltitudeMeters() * 3.28084;
        return heightOutput;
    }

    readFloatHumidity(){
            // Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
    // Output value of "47445" represents 47445/1024 = 46. 333 %RH
    let buffer: number[] = this.readRegisterRegion(BME280_ADDRESS, BME280_HUMIDITY_MSB_REG, 2);
    this.readRegisterRegion(BME280_ADDRESS, BME280_HUMIDITY_MSB_REG, 2);
    let adc_H: number = (buffer[0] << 8) | (buffer[1]);
    
        let var1: number = this.t_fine - (76800);
    
    var1 = (((((adc_H << 14) - ((this.calibration.dig_H4) << 20) - ((this.calibration.dig_H5) * var1)) + (16384)) >> 15) * (((((((var1 * (this.calibration.dig_H6)) >> 10) * (((var1 * (this.calibration.dig_H3)) >> 11) + (32768))) >> 10) + (2097152)) * (this.calibration.dig_H2) + 8192) >> 14));
    var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * (this.calibration.dig_H1)) >> 4));
    var1 = (var1 < 0 ? 0 : var1);
    var1 = (var1 > 419430400 ? 419430400 : var1);

    return (var1>>12) / 1024.0;
    }

//Temperature related 
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC.
    // t_fine carries fine temperature as global value

    //get the reading (adc_T);
    readTempC(){
        let buffer: number[] = [0,0,0]
        buffer[0] = this.readRegister(BME280_ADDRESS, BME280_TEMPERATURE_MSB_REG);
        buffer[1] = this.readRegister(BME280_ADDRESS, BME280_TEMPERATURE_LSB_REG);
        buffer[2] = this.readRegister(BME280_ADDRESS, BME280_TEMPERATURE_XLSB_REG);
        let adc_T = (buffer[0] << 12) | (buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);

        let var1: number = (((adc_T>>3) - (this.calibration.dig_T1<<1)) * (this.calibration.dig_T2)) >> 11;
        let var2: number = (((((adc_T>>4) - (this.calibration.dig_T1)) * ((adc_T>>4) - (this.calibration.dig_T1))) >> 12) * (this.calibration.dig_T3)) >> 14;

        this.t_fine = var1 + var2;
        let output: number = (this.t_fine * 5 + 128) >> 8;

        output = output / 100 + this.BMEtempCorrection;
        return output;
    }


    readTempF(){
        let output = this.readTempC();
        output = (output * 9) / 5 + 32;

        return output;
    }


//Dewpoint related methods
//From Pavel-Sayekat: https://github.com/sparkfun/SparkFun_BME280_Breakout_Board/pull/6/files
    dewPointC(){
        let celsius: number = this.readTempC(); 
        let humidity: number = this.readFloatHumidity();
        // (1) Saturation Vapor Pressure = ESGG(T)
        let RATIO: number = 373.15 / (273.15 + celsius);
        let RHS: number = -7.90298 * (RATIO - 1);

        RHS += 5.02808 * Math.log(RATIO) / Math.log(10);
        RHS += -1.3816e-7 * (Math.pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
        RHS += 8.1328e-3 * (Math.pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
        RHS += Math.log(1013.246) / Math.log(10);
         // factor -3 is to adjust units - Vapor Pressure SVP * humidity
        let VP: number = Math.pow(10, RHS - 3) * humidity;
         // (2) DEWPOINT = F(Vapor Pressure)
        let T: number = Math.log(VP/0.61078);   // temp var
        return (241.88 * T) / (17.558 - T);
    }

    dewPointF(){
        return (this.dewPointC() * 1.8 + 32); //Convert C to F
    }

    checkForStatusError(){
        let value = this.readRegister(CCS811_ADDRESS, CCS811_STATUS);
        return (value & 1 << 0);
    }
    appValid(){
        let value = this.readRegister(CCS811_ADDRESS, CCS811_STATUS);
        return (value & 1 << 4);
    }

//The following utilities read and write

//this.readRegisterRegion takes a unumber array address as input and reads
//a chunk of memory into that array.

    readRegisterCalibration(address: number, offset: number) {
        pins.i2cWriteNumber(address, offset, NumberFormat.UInt8BE);
        return pins.i2cReadNumber(address, NumberFormat.Int8LE);
    }

    readRegisterRegion(address: number, offset: number, length: number){
        let data: number[] = [];
        pins.i2cWriteNumber(address, offset, NumberFormat.UInt8LE, false);
        pause(50)
        for (let i = 0; i < length-1; i++){
            data.push(pins.i2cReadNumber(address, NumberFormat.UInt8LE, true))
        }
        data.push(pins.i2cReadNumber(address, NumberFormat.UInt8LE, false))
        return data;

    }

    readRegisterRegion32(address: number, offset: number, length: number){
        let data: number[] = [];
        pins.i2cWriteNumber(address, offset, NumberFormat.UInt8LE, false);
        pause(50)
        for (let i = 0; i < length-1; i++){
            data.push(pins.i2cReadNumber(address, NumberFormat.Int32LE, true))
        }
        data.push(pins.i2cReadNumber(address, NumberFormat.Int32LE, false))
        return data;
    }

//this.readRegister reads one register
    readRegister(address: number, offset: number){
        pins.i2cWriteNumber(address, offset, NumberFormat.UInt8LE, false);
        pause(50)
        let value = pins.i2cReadNumber(address, NumberFormat.UInt8LE, false);
        return value;
    }

    readRegisterSigned(address: number, offset: number){
        pins.i2cWriteNumber(address, offset, NumberFormat.Int8LE, false);
        pause(50)
        let value = pins.i2cReadNumber(address, NumberFormat.Int8LE, false);
        return value;
    }

//Reads two regs, LSByte then MSByte order, and concatenates them
//Used for two-byte reads
    readRegisterInt16(address: number, offset: number){
        pins.i2cWriteNumber(address, offset, NumberFormat.UInt8LE, false);
        pause(50)
        return pins.i2cReadNumber(address, NumberFormat.UInt16BE, false);
    }

    readRegisterInt8LE(address: number, offset: number){
        pins.i2cWriteNumber(address, offset, NumberFormat.UInt8BE);
        return pins.i2cReadNumber(address, NumberFormat.Int8LE);
    }

    readRegisterUInt8LE(address: number, offset: number){
        pins.i2cWriteNumber(address, offset, NumberFormat.UInt8BE);
        return pins.i2cReadNumber(address, NumberFormat.UInt8LE);
    }
//Writes a byte;
    writeRegister(address: number, offset: number, value: number){
        let message = (offset << 8) | value;
        pins.i2cWriteNumber(address, message, NumberFormat.UInt16BE, false);
        pause(50)
        return;
    }

    writeRegisterShort(address: number, offset: number){
        pins.i2cWriteNumber(address, offset, NumberFormat.UInt8LE, false);
        pause(50)
        return;
    }

    multiWriteRegisterLE(address: number, offset: number, values: number[], length: number){
        for (let i=0; i < length-1; i++){
            let message = (offset << 8) | values[i];
            pins.i2cWriteNumber(address, message, NumberFormat.UInt16LE, true);
            pause(50)
        }
        let message = (offset << 8) | values[length-1];
        pins.i2cWriteNumber(address, values[length-1], NumberFormat.UInt16LE, false);
        pause(50)
        return;

    }

    multiWriteRegisterBE(address: number, offset: number, values: number[], length: number){
        for (let i=0; i < length-1; i++){
            let message = (offset << 8) | values[i];
            pins.i2cWriteNumber(address, message, NumberFormat.UInt16BE, true);
            pause(50)
        }
        let message = (offset << 8) | values[length-1];
        pins.i2cWriteNumber(address, values[length-1], NumberFormat.UInt16BE, false);
        pause(50)
        return;

    }

// private

    //Validates an over sample value
    //Allowed values are 0 to 16
    //These are used in the humidty, pressure, and temp oversample functions
    checkSampleValue(userValue: number){
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
    } //Checks for valid over sample values


}

class SensorCalibration {

    dig_T1: number;
    dig_T2: number;
    dig_T3: number;
    
    dig_P1: number;
    dig_P2: number;
    dig_P3: number;
    dig_P4: number;
    dig_P5: number;
    dig_P6: number;
    dig_P7: number;
    dig_P8: number;
    dig_P9: number;
    
    dig_H1: number;
    dig_H2: number;
    dig_H3: number;
    dig_H4: number;
    dig_H5: number;
    dig_H6: number;

    constructor(){
        return;
    }
}








