/**
 * Michael Schneider and James Luther @ University of Colorado Boulder
 * August 12, 2022
 * 
 * Typescript version of pxt-gator-environment extension from Sparkfun.
 */


/**
 * Functions to operate the gatorEnvironment sensor
 */
enum MeasurementType {
    degreesC = 1,
    degreesF = 2,
    humidity = 3,
    pressure = 4,
    eCO2 = 5,
    TVOC = 6,
}

//% color=#f44242 icon="\uf0c2"
namespace gatorEnvironment {
    
    let gatorEnvironmentCombo = new Environment();

    /**
    * Initialize the gator:environment sensor for readings
    */
    //% weight=32 
    //% blockId="gatorEnvironment_beginEnvironment" 
    //% block="initialize gator:environment sensors"
    export function beginEnvironment() {
        gatorEnvironmentCombo.begin();
        return
    }

    /**
    * Grab the temperature, humidity, pressure, equivalent C02, or total Volatile Organic Compounds from the gator:environment
    */
    //% weight=31
    //% blockId="gatorEnvironment_measurement"
    //% block="get %MeasurementType | value"
    export function measurement(type: MeasurementType): number {
        let value = 0;
        switch (type) {
            case 1:
                value = gatorEnvironmentCombo.readTempC();
                break;
            case 2:
                value = gatorEnvironmentCombo.readTempF();
                break;
            case 3:
                value = gatorEnvironmentCombo.readFloatHumidity();
                break;
            case 4:
                value = gatorEnvironmentCombo.readFloatPressure();
                break;
            case 5:
                if (gatorEnvironmentCombo.dataAvailable()) {
                    gatorEnvironmentCombo.readAlgorithmResults();
                }
                value = gatorEnvironmentCombo.getCO2();
                break;
            case 6:
                if (gatorEnvironmentCombo.dataAvailable()) {
                    gatorEnvironmentCombo.readAlgorithmResults();
                }
                value = gatorEnvironmentCombo.getTVOC();
                break;
            default:
                value = 0;
                break;
        }
        return value;
    }
}


