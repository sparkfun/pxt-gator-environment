# gator:environment Temperature, Pressure, Humidity, CO2 and TVOC Sensor

[![Community Discord](https://img.shields.io/discord/448979533891371018.svg)](https://aka.ms/makecodecommunity)

The gator:environment, capable of reading several qualities about the air can be purchased [here.](https://www.sparkfun.com/products/15269). The SparkFun gator:environment micro:bit Accessory Board utilizes the popular CCS811 and BME280 ICs to take care of all of your environmental readings of atmospheric quality. This sensor board can provide a variety of environmental data, including barometric pressure, humidity, temperature, equivalent TVOCs and equivalent CO2 (or eCO2) levels. The gator:environment connects to the SparkFun gator:bit via alligator-clip cables, allowing for easy access to the micro:bit's I2C pins.
![SparkFun gator:environment](https://raw.githubusercontent.com/sparkfun/pxt-gator-environment/master/icon.png)  

## ~ hint

To use this package, go to https://makecode.microbit.org, click ``Add package`` and search for **gator-environment**. The package is located [here](https://cdn.sparkfun.com/assets/learn_tutorials/8/7/3/pxt-gator-environment-package.hex)

## ~

## Basic usage

```blocks
//Initializes our environmental sensor so we can get data from it.
gatorEnvironment.beginEnvironment()
```

Use ``||Use Initialize gator:Environment sensors||`` to initialize the sensors on the gator:Environment board.

```blocks
//Use measurement to grab data from any of the sensors channels.
gatorEnvironment.measurement(MeasurementType.degreesC)
```

Use ``||degreesC value||`` to grab values from the board. Changing what value you want is as easy as selecting it from the drop down menu.

## Example: Thermometer

```blocks
//Read degrees celsius and write it to the micro:bit screen as a number
gatorEnvironment.beginEnvironment()
basic.forever(function () {
    serial.writeLine("" + gatorEnvironment.measurement(MeasurementType.degreesC))
})
```

## Supported targets

* for PXT/microbit

## License

MIT

```package
gatorEnvironment=github:sparkfun/pxt-gator-environment
```
