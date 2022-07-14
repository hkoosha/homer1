## What?

A project to read out sensors connected to ESP32 and write to InfluxDB. ESP-IDF based.

## Sensors:

- BMP180
- SHT3X
- S8 (SenseAir)
- PMS5003
- CSS811

## Warning!

- Not necessarily power consumption friendly.
- SHT3X takes no calibration.
- Sensor modes are not configurable, everything about sensor configuration is hard-coded.
- If the Prometheus people find out the way I have written exporters, they'll cry.
- Want something proper? have a look at https://gitlab.com/UncleRus/esp-idf-lib

#### References

Some BMP180 code stolen from:
[krzychb/bmp180-tests](https://github.com/krzychb/bmp180-tests) [APACHE-2.0]

Some S8 code stolen from:
[SFeli/ESP32_S8](https://github.com/SFeli/ESP32_S8/blob/master/ESP32_S8_01.ino)

Some Prometheus ideas got from:
[polyfloyd/esp32-sensornode](https://github.com/polyfloyd/esp32-sensornode)

## Screenshots

InfluxDB:

![InfluxDB](screenshots/influxdb.png)
<br>
<br>

Grafana:

![Grafana](screenshots/grafana.png)
<br>
<br>

Serial output:

![esp32 output](screenshots/esp32_output.png)

