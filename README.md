# gator:environment Temperature, Pressure, Humidity, CO2 and TVOC Sensor

[![Community Discord](https://img.shields.io/discord/448979533891371018.svg)](https://aka.ms/makecodecommunity)

The gator:environment, capable of reading several qualities about the air can be purchased [here.](https://www.sparkfun.com/products/15269)
The gator:light, included on the gator:starter is an analog light sensor that can be alligator clipped to the micro:bit or gator:bit board.

![SparkFun gator:light](https://raw.githubusercontent.com/sparkfun/pxt-gator-environment/master/icon.png)  

## ~ hint

To use this package, go to https://makecode.microbit.org, click ``Add package`` and search for **gator-environment**. The package is located [here](https://makecode.microbit.org/pkg/sparkfun/pxt-gator-environment)

## ~

## Basic usage

```blocks
//Initializes our environmental sensor so we can get data from it.
gatorEnvironment.beginEnvironment()
```

Use ``||Use Initialize gator:Environment sensors||`` to initialize the sensors on the gator:Environment board.

```blocks
//Use getMeasurement to grab data from any of the sensors channels.
gatorEnvironment.getMeasurement(MeasurementType.degreesC)
```

Use ``||Get degreesC value||`` to grab values from the board. Changing what value you want is as easy as selecting it from the drop down menu.

## Example: Thermometer

```blocks
//Read degrees celsius and write it to the micro:bit screen as a number
gatorEnvironment.beginEnvironment()
basic.forever(function () {
    serial.writeLine("" + gatorEnvironment.getMeasurement(MeasurementType.degreesC))
})
```

## Supported targets

* for PXT/microbit

## License

MIT

```package
gatorEnvironment=github:sparkfun/pxt-gator-environment
```
