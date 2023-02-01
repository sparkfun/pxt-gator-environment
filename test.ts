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

gatorEnvironment.beginEnvironment()
basic.forever(function () {
    serial.writeLine("" + gatorEnvironment.measurement(MeasurementType.degreesC))
    serial.writeLine("" + gatorEnvironment.measurement(MeasurementType.degreesF))
    serial.writeLine("" + gatorEnvironment.measurement(MeasurementType.humidity))
    serial.writeLine("" + gatorEnvironment.measurement(MeasurementType.pressure))
    serial.writeLine("" + gatorEnvironment.measurement(MeasurementType.eCO2))
    serial.writeLine("" + gatorEnvironment.measurement(MeasurementType.TVOC))
})