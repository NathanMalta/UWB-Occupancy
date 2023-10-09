#include <Adafruit_GPS.h>
#include <Adafruit_PMTK.h>
#include <NMEA_data.h>

/*
  Arduino LSM6DSOX - Simple Accelerometer

  This example reads the acceleration values from the LSM6DSOX
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano RP2040 Connect

  created 10 May 2021
  by Arturo Guadalupi

  This example code is in the public domain.
*/
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Arduino_LSM6DSOX.h>
#include <SPI.h>
#include <SdFat.h>

//create gyro + accel
TwoWire dev_i2c(&sercom3, SDA, SCL);  
LSM6DSOXClass imu(dev_i2c, 107);

//create magnetometer
Adafruit_LIS2MDL lis2mdl(12345);

//create sd card objects
SdFile logFile;
SdFat sd;
int SDChipSelect = 10;

//create GPS objects
Adafruit_GPS GPS(&Serial1);
unsigned long gpsTimer = millis();
int GPS_update = 2000; //log / print updates every 2 sec.


char logFilename[] = "output.txt";

unsigned long loopNum = 0;
int loopsPerLog = 10;
bool logSerial = false;

void setup() {
  Serial.begin(9600);
  while (!Serial); //wait for serial before printing

  //can we connect to gyro + accel?
  if (!imu.begin()) {
    Serial.println("Could not connect to LSM6DSOX (Gyro + Accel)!");
    while (1) delay(10);
  } else {
    Serial.println("Found LSM6DSOX (Gyro + Accel).");
  }

  //can we connect to magnetometer?
  lis2mdl.enableAutoRange(true);
  if (!lis2mdl.begin()) {  // I2C mode
    Serial.println("Could not connect to LIS2MDL (Magnetometer)!");
    while (1) delay(10);
  } else {
    Serial.println("Found LIS2MDL (Magnetometer).");
  }

  //can we connect to SD Card?
  if (!sd.begin(SDChipSelect, SD_SCK_MHZ(4))) {
    Serial.println("SDCard Initialization failed!");
    while (1) delay(10);
  }
  else{
      Serial.println("SDCard Initialization done.");
  }

  //create the logfile (if it doesn't exist)
  if (!logFile.open(logFilename, O_CREAT | O_WRITE)) {
    Serial.println("SDCard log file creation failed!");
    while (1) delay(10);
  }

  //startup GPS module
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
}

void loop() {
  loopNum++;

  float xAccel, yAccel, zAccel;
  float xGyro, yGyro, zGyro;
  float xMag, yMag, zMag;
  float temp;
  sensors_event_t event;

  if (imu.accelerationAvailable() && imu.gyroscopeAvailable() && imu.temperatureAvailable()) {
    imu.readAcceleration(xAccel, yAccel, zAccel);
    imu.readGyroscope(xGyro, yGyro, zGyro);
    imu.readTemperatureFloat(temp);

    lis2mdl.getEvent(&event);
    xMag = event.magnetic.x;
    yMag = event.magnetic.y;
    zMag = event.magnetic.z;

    printIMUInfo(xAccel, yAccel, zAccel, xGyro, yGyro, zGyro, xMag, yMag, zMag, temp, (loopNum % loopsPerLog) == 0);
  }

  //update GPS Sensor
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (false)
    if (c) Serial.print(c);

  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    // Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  //print if needed
  if (millis() - gpsTimer >= GPS_update) {
    gpsTimer = millis();

    Serial.println("------- GPS INFO: -------");
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);

    //check GPS status
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
      Serial.println();
    } else{
      Serial.println("No GPS fix. :(");
    }
  }

}

void printIMUInfo(float xAccel, float yAccel, float zAccel, float xGyro, float yGyro, float zGyro, float xMag, float yMag, float zMag, float temp, bool logSD) {
  if (logSerial) {
    Serial.print("Accelerometer:\t");
    Serial.print(xAccel);
    Serial.print("\t");
    Serial.print(yAccel);
    Serial.print("\t");
    Serial.print(zAccel);
    Serial.print("\t");

    Serial.print("Gyroscope:\t");
    Serial.print(xGyro);
    Serial.print("\t");
    Serial.print(yGyro);
    Serial.print("\t");
    Serial.print(zGyro);
    Serial.print("\t");

    Serial.print("Magnetometer:\t");
    Serial.print(xMag);
    Serial.print("\t");
    Serial.print(yMag);
    Serial.print("\t");
    Serial.print(zMag);
    Serial.print("\t");

    Serial.print("Temp:\t");
    Serial.print(temp);
    Serial.println();
  }

  if (logSD) {
    //can we create a log file
    if(!logFile.open(logFilename, O_APPEND | O_WRITE)) {
        Serial.println("Could not create log file.  Check SD Card.");
    }
    
    //log sensor info
    logFile.print(millis());
    logFile.print(",");
    logFile.print(xAccel);
    logFile.print(",");
    logFile.print(yAccel);
    logFile.print(",");
    logFile.print(zAccel);
    logFile.print(",");

    logFile.print(xGyro);
    logFile.print(",");
    logFile.print(yGyro);
    logFile.print(",");
    logFile.print(zGyro);
    logFile.print(",");

    logFile.print(xMag);
    logFile.print(",");
    logFile.print(yMag);
    logFile.print(",");
    logFile.print(zMag);
    logFile.print(",");

    logFile.print(temp);
    logFile.println();

    logFile.close();
  }
}
