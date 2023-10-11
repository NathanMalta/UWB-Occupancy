#include <Adafruit_GPS.h>
#include <Adafruit_PMTK.h>
#include <NMEA_data.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Arduino_LSM6DSOX.h>
#include <SPI.h>
#include <SdFat.h>
#include <DW1000.h>
#include "DW1000Ranging.h"

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

//create NeoPixel objects
Adafruit_NeoPixel pixels(1, 8, NEO_GRB + NEO_KHZ800);

//logging info
char logFilename[] = "output.txt";
unsigned long loopNum = 0;
int loopsPerLog = 10;
bool logSerial = false;

//DW1000 (UWB Chip) constants
const bool IS_ANCHOR = true;
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 17; // irq pin
const uint8_t PIN_SS = 19; // spi select pin

void setup() {
  Serial.begin(9600);

  //Startup neopixel
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(255, 255, 0)); //yellow for init
  pixels.setBrightness(10);
  pixels.show();

  while (!Serial); //wait for serial before printing

  //can we connect to gyro + accel?
  if (!imu.begin()) {
    Serial.println("Could not connect to LSM6DSOX (Gyro + Accel)!");
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.setBrightness(10);
    pixels.show();
    while (1) delay(10);
  } else {
    Serial.println("Found LSM6DSOX (Gyro + Accel).");
  }

  //can we connect to magnetometer?
  lis2mdl.enableAutoRange(true);
  if (!lis2mdl.begin()) {  // I2C mode
    Serial.println("Could not connect to LIS2MDL (Magnetometer)!");
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.setBrightness(10);
    pixels.show();
    while (1) delay(10);
  } else {
    Serial.println("Found LIS2MDL (Magnetometer).");
  }

  //can we connect to SD Card?
  if (!sd.begin(SDChipSelect, SD_SCK_MHZ(4))) {
    Serial.println("SDCard Initialization failed!");
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.setBrightness(10);
    pixels.show();
    while (1) delay(10);
  }
  else{
      Serial.println("SDCard Initialization done.");
  }

  //create the logfile (if it doesn't exist)
  if (!logFile.open(logFilename, O_CREAT | O_WRITE)) {
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.setBrightness(10);
    pixels.show();
    Serial.println("SDCard log file creation failed!");
    while (1) delay(10);
  }

  //startup GPS module
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

  //startup UWB module (DW1000)
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  if (IS_ANCHOR) {
    DW1000Ranging.attachBlinkDevice(newBlink);
    DW1000Ranging.startAsAnchor((char*)"82:17:5B:D5:A9:9A:E2:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY);
    Serial.println("Starting up device in Anchor mode");
  } else {
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.startAsTag((char*)"7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY);
    Serial.println("Starting up device in Tag mode");
  }

}

void loop() {
  //update DW1000
  DW1000Ranging.loop();

  //record measurements from other sensors
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
    //check GPS status
    if (GPS.fix) {
      printGPSInfo(GPS.latitudeDegrees, GPS.longitudeDegrees, GPS.speed, GPS.altitude, GPS.satellites);
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(0, 255, 0));
      pixels.setBrightness(10);
      pixels.show();
    } else{
      Serial.println("No GPS fix. :(");
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(0, 0, 255));
      pixels.setBrightness(10);
      pixels.show();
    }
  }

}

void printGPSInfo(float lat, float lon, float speed, float altitude, int numSats) {

  //output to serial
  Serial.println("------- GPS INFO: -------");
  Serial.print("Location: ");
  Serial.print(lat, 4);
  Serial.print(", ");
  Serial.print(lon, 4);
  Serial.println();
  Serial.print("Speed (knots): "); Serial.println(speed);
  Serial.print("Altitude: "); Serial.println(altitude);
  Serial.print("Satellites: "); Serial.println((int)numSats);
  Serial.println();

  //output to SD
  noInterrupts();
  if(!logFile.open(logFilename, O_APPEND | O_WRITE)) {
    Serial.println("Could not create log file.  Check SD Card.");
  }
  logFile.print("GPS: ");
  logFile.print(millis());
  logFile.print(",");
  logFile.print(lat, 5);
  logFile.print(",");
  logFile.print(lon, 5);
  logFile.print(",");
  logFile.print(speed, 5);
  logFile.print(",");
  logFile.print(altitude, 5);
  logFile.print(",");
  logFile.print((int)numSats);
  logFile.println();
  logFile.close();
  interrupts();

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
    noInterrupts();
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
    interrupts();
  }
}

void newRange() {
  Serial.print("from: "); Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print("\t Range: "); Serial.print(DW1000Ranging.getDistantDevice()->getRange()); Serial.print(" m");
  Serial.print("\t RX power: "); Serial.print(DW1000Ranging.getDistantDevice()->getRXPower()); Serial.println(" dBm");

  noInterrupts();
  if(!logFile.open(logFilename, O_APPEND | O_WRITE)) {
        Serial.println("Could not create log file.  Check SD Card.");
  }
  logFile.print("UWB: ");
  logFile.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  logFile.print(",");
  logFile.print(DW1000Ranging.getDistantDevice()->getRange());
  logFile.print(",");
  logFile.print(DW1000Ranging.getDistantDevice()->getRXPower());
  logFile.println();
  logFile.close();
  interrupts();

  Serial.println();
}

//This is for Anchors only
void newBlink(DW1000Device* device) {
  Serial.print("blink; 1 device added ! -> ");
  Serial.print(" short:");
  Serial.println(device->getShortAddress(), HEX);
}

//This is for Tags only
void newDevice(DW1000Device* device) {
  Serial.print("ranging init; 1 device added ! -> ");
  Serial.print(" short:");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}



