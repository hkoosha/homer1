## What?
A project to read out these sensors connected to ESP32 and print over serial.
ESP-IDF based.


## Warning!
- Not necessarily power consumption friendly.
- SHT3X takes no calibration.
- Sensor modes are not configurable, everything about sensor configuration is hard-coded.
- It's more of a C-ish code put into C++.

## Sensors:
- BMP180
- SHT3X
- S8 (Senseair)
- PMS5003




Some BMP180 code stolen from: https://github.com/krzychb/bmp180-tests [APACHE-2.0]

Some S8 code stolen from: https://github.com/SFeli/ESP32_S8/blob/master/ESP32_S8_01.ino

