# ArduGrowBox
This is an Arduino project that aims for creating a controller and datalogger for temperature and humidity of a plant growbox, along automatic watering of up to one plant. It is primarily intended for the microcontroller Arduino Nano 3.0, widely available in Aliexpress.

## About

### Some of its functions are:

* Display of relative humidity, temperature and leaf vapor pressure (VPD).

* Control of soil humidity.

* Automatic watering below a certain threshold of soil humidity.

* Datalogging into a microSD card the humidity, temperature, VPD, soil humidity, and watering, along the time of measure.



### Future functions are:
* Basic control of Extractor and intake fan speeds according to VPD values.

* Basic control of a humidifier module according to humidity values.

## Required modules and Connections
### External modules
The project requires some modules that can be bought all in Aliexpress, are widely available and have extensive libraries.
* LM2596S adjustable buck converter
* TB6612FNG Motor Driver
  * Used for the extractor and intake fans
* DHT22 Air and Humidity reader
* Capacitative Soil Moisture Sensor v2.0
  * <sub> The quality varies widely in Aliexpress. Found the most reliable the V2.0 from a seller vs the V1.2 from Tenstar Robots </sub>
* DS3231 RTC
  * Used to keep track on the time even if power outage.
  * Allows adding the time into the datalogging files to keep track
* LCD1602+PCF8574 I2C
  * This is the typical 16x2 LCD screen with I2C mounted on the back to use only 4 wires.
  * cheaply and widely available on Aliexpress too.
* TF SD CARD MODULE
  * Typical SD card module
* 3x1 Matrix Keypad
  * <sub> mine is kinda shitty and can be activated by slight touch... maybe consider another option </sub>

## Wiring diagram

![Electrical diagram in EasyEDA of the project](https://github.com/LoloCG/ArduGrowBox/blob/6193741d591181d0ca1107c937eb108ad0e33349/Nano%20V1/Schematic_AutoGrowBox_Arduino_2024-02-19.png)

## Before going into the code...
This is a still in development first project for someone who just has started using microcontrollers.
Expect bad coding among other things...


# Future of the project

After having trouble trying to implement all my desired functionalities into an Arduino nano, due to memory constrains of the microcontroller ATMega328p, my next aim would be to recreate a better working version based around expressiv ESP32 microcontrollers. My most likely candidate is ESP32-S3 MCU, allowing greater memory, more I/O pins, internet and bluetooth connectivity.
