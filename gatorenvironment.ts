/**
* Andy England @ SparkFun Electronics
* September 6, 2018

* Development environment specifics:
* Written in Microsoft Makecode
* Tested with a SparkFun gatorEnvironment sensor and micro:bit
*
* This code is released under the [MIT License](http://opensource.org/licenses/MIT).
* Please review the LICENSE.md file included with this example. If you have any questions
* or concerns with licensing, please contact techsupport@sparkfun.com.
* Distributed as-is; no warranty is given.
*/


/**
 * Functions to operate the gatorEnvironment sensor
 */

 enum measurementType{
	 degreesC=1,
	 degreesF=2,
	 humidity=3,
	 pressure=4,
	 eCO2=5,
	 TVOC=6,
 } 
 
//% color=#f44242 icon="\uf185"
namespace gatorEnvironment {
    // Functions for reading Particle from the gatorEnvironment in Particle or straight adv value
	
	//% weight=31 
	//% blockId="gatorEnvironment_begin" 
	//% block="Initialize gator:Environment sensors"
	//% shim=gatorEnvironment::begin
	export function begin(){
		return
	}
	
	//% weight=30
	//% blockId="gatorEnvironment_getMeasurement"
	//% block="Get %measurementType | value"
	//% shim=gatorEnvironment::getMeasurement
	export function getMeasurement(type: measurementType): number{
		return 0
	}
}