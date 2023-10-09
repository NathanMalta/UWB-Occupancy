# UWB Bus Occupancy Sensing

This project aims to use [Ultra-Wideband](https://en.wikipedia.org/wiki/Ultra-wideband) (UWB) signals to sense the number of people aboard a bus.  This project is a part of Georgia Tech's [CS8803 - Mobile Computing and IoT](https://faculty.cc.gatech.edu/~dhekne/cs8803/course.html) taught by Prof. Ashutosh Dhekne.

This project references code from Shivam Agarwal [here](https://github.com/shivamag437/cs8803_mci)

## Configure Arduino IDE to work with Adafruit's Feather
Add the board manager:
File > Preferences > Additional Board Manager URLS, then copy/paste the Adafruit board url:
```
https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
```

Install the Feather M0 Board:
Tools > Board > Board Manager
Then, search and install "Adafruit SAMD Boards by Adafruit" and "Arduino SAMD Boards (32-bits ARM Cortex-M0+) by Arduino

## Installing Libraries
Tools > Manage Libraries > Search and install the following libraries:
```
Arduino_LSM6DSOX - Control of the LSM6DSOX Gyroscope + Accelerometer
Adafruit LIS2MDL - Control of the LIS2MDL Magnetometer
Adafruit GPS Library - Control of the PA1616D GPS Module
Adafruit NeoPixel - Control of the status LED
```
