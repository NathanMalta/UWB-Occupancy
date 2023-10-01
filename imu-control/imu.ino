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

//init gyro + accel
TwoWire dev_i2c(&sercom3, SDA, SCL);  
LSM6DSOXClass imu(dev_i2c, 107);

//init magnetometer
Adafruit_LIS2MDL lis2mdl(12345);

void setup() {
  Serial.begin(9600);
  while (!Serial);

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

}

void loop() {
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

    printIMUInfo(xAccel, yAccel, zAccel, xGyro, yGyro, zGyro, xMag, yMag, zMag, temp);
  }
}

void printIMUInfo(float xAccel, float yAccel, float zAccel, float xGyro, float yGyro, float zGyro, float xMag, float yMag, float zMag, float temp) {
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
