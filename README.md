# ArduGrowBox

## About
This is an Arduino project that aims for creating a controller and datalogger for temperature and humidity of a plant growbox, along automatic watering of up to one plant. It is primarily intended for the microcontroller Arduino Nano 3.0, widely available in Aliexpress.

Some of its functions are:
·Display of relative humidity, temperature and leaf vapor pressure (VPD).
·Control of soil humidity.
·Automatic watering below a certain threshold of soil humidity.
·Datalogging into a microSD card the humidity, temperature, VPD, soil humidity, and watering, along the time of measure.

Future functions are:
·Basic control of Extractor and intake fan speeds according to VPD values.
·Basic control of a humidifier module according to humidity values.

## Before into the code...
This is a still in development first project for someone who just has started using microcontrollers.
Expect bad coding among other things...

# Future of the project

After having trouble trying to implement all my desired functionalities into an Arduino nano, due to memory constrains of the microcontroller ATMega328p, my next aim would be to recreate a better working version based around expressiv ESP32 microcontrollers. My most likely candidate is ESP32-S3 MCU, allowing greater memory, more I/O pins, internet and bluetooth connectivity.
