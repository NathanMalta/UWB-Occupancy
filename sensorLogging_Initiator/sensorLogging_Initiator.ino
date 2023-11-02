#include <Adafruit_GPS.h>
#include <Adafruit_PMTK.h>
#include <NMEA_data.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <Arduino_LSM6DSOX.h>
#include <SdFat.h>
#include <DW1000.h>
#include "DW1000Ranging.h"

#include <SPI.h>
#include <math.h>
#include "genericFunctions.h"
#include "Adafruit_LSM9DS1.h"
#include <time.h>
#include<TimeLib.h>
#include "RTClib.h"
#include<Wire.h>


#define INITIATOR 1
#define LOG_IMU 1
#define LOG_GPS 1

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

//-------------- UWB Profile Constants --------------

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define DEBUG_PRINT 0
#define DEBUG_CIR 0
// connection pins
#define OUR_UWB_FEATHER 1
#define NCIR_FULL 1016
#define RX_TIME_FP_INDEX_OFFSET 5
int packet_count = 1;
int packet_type = 0;

int UWB_send_timer = 10; // when to send. 
unsigned long uwbTimer = millis();

//DW1000 (UWB Chip) constants
#if(OUR_UWB_FEATHER==1)
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 17; // irq pin
const uint8_t PIN_SS = 19; // spi select pin
#endif

// DEBUG packet sent status and count
volatile boolean received = false;
volatile boolean error = false;
volatile int16_t numReceived = 0; // todo check int type
volatile boolean sendComplete = false;
volatile boolean RxTimeout = false;
String message;

byte tx_poll_msg[MAX_POLL_LEN] = {POLL_MSG_TYPE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

byte rx_packet[128];
uint8_t myAcc[1000];

typedef enum states{STATE_IDLE, STATE_POLL} STATES;
volatile uint8_t current_state = STATE_IDLE;
unsigned long silenced_at =0;
#define SILENCE_PERIOD 120
long randNumber;
int currentSlots = 8;
int myDevID = INITIATOR+2;

#define MAX_TIMEOUTS 2
volatile boolean timeout_established[MAX_TIMEOUTS];
volatile boolean timeout_triggered[MAX_TIMEOUTS];
volatile boolean timeout_overflow[MAX_TIMEOUTS];
volatile uint64_t timeout_time[MAX_TIMEOUTS];

// int FIXED_DELAY = 3;

//Function to receive a packet
void receiver(uint16_t rxtoval=0 ) {
  received = false;
  DW1000.newReceive();
  DW1000.setDefaults();
  // we cannot don't need to restart the receiver manually
  DW1000.receivePermanently(false);
  if (rxtoval>0) {
    DW1000.setRxTimeout(rxtoval);
  } else {
    //Serial.print("Resetting Timeout to  ");
    //Serial.println(rxtoval);
    DW1000.setRxTimeout(rxtoval);
  }
  DW1000.startReceive();
  //Serial.println("Started Receiver");
}

void print_CIR()
{
    char buff[140];
    char long_buff[1400];
  
    byte firstPath[2];
    DW1000.readBytes(RX_TIME, RX_TIME_FP_INDEX_OFFSET, firstPath, 2);
    uint16_t firstpath = uint16_t(firstPath[1]<<8 | firstPath[0]);
    uint16_t firstPath_integer = (firstpath & 0xFFC0) >> 6;
    uint16_t firstPath_fraction = (firstpath & 0x003F);
    float RX_POWER = DW1000.getReceivePower();
    float FP_POWER = DW1000.getFirstPathPower();
    
    uint8_t myAcc[4 * NCIR_FULL + 6];
    int starttap = 0; // 720
    int endtap = 1015; // 816 
    int16_t RealData = 0;
    int16_t ImaginaryData = 0;
    
    sprintf(long_buff, "CIR %d %d ", firstPath_fraction, firstPath_integer);
    DW1000.getAccMem(myAcc, 0, endtap + 1); //myAcc will contain 16 bit real imaginary pairs
    
    for (int i = starttap ; i < endtap + 1; i++) {
      RealData = myAcc[(i * 4) + 2] << 8 | myAcc[(i * 4) + 1];
      ImaginaryData = myAcc[(i * 4) + 4] << 8 | myAcc[(i * 4) + 3];
   sprintf(buff, "[%d,%d,%d,%d],", packet_count, RealData, ImaginaryData, i + 1);
    // strcat(long_buff, buff);
    Serial.print(buff);
  }
  Serial.println();
  // Serial.println(long_buff);
}

// ------------- DEVICE SETUP -------------

void setup() {
  analogReadResolution(10);
  Serial.begin(115200);

  //Startup neopixel
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(255, 255, 0)); //yellow for init
  pixels.setBrightness(10);
  pixels.show();

  //while (!Serial); //wait for serial before printing

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
  //DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  //DW1000Ranging.attachNewRange(newRange);
  //DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // if (IS_ANCHOR) {
  //   DW1000Ranging.attachBlinkDevice(newBlink);
  //   DW1000Ranging.startAsAnchor((char*)"82:17:5B:D5:A9:9A:E2:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY);
  //   Serial.println("Starting up device in Anchor mode");
  // } else {
  //   DW1000Ranging.attachNewDevice(newDevice);
  //   DW1000Ranging.startAsTag((char*)"7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY);
  //   Serial.println("Starting up device in Tag mode");
  // }

  // Start UWB CIR
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(6);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  // attach callback for (successfully) received messages
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveTimeoutHandler(handleRxTO);
  DW1000.attachReceiveFailedHandler(handleError);
  DW1000.attachErrorHandler(handleError);
  DW1000.attachSentHandler(handleSent);
  // start reception
  
  #if (INITIATOR==0)
    receiver(0);
    
  #endif
  current_state = STATE_IDLE;

  for(int i=0;i<MAX_TIMEOUTS;i++) {
    timeout_established[i]=false;
    timeout_triggered[i]=false;
    timeout_overflow[i]=false;
    timeout_time[i]=0;
  } 
}

// --------------- DEFINE CALLBACK FUNCTIONS ---------------

void handleSent() {
  // status change on sent success
  sendComplete = true;
  //Serial.println("Send complete");
}


void handleReceived() {
  // status change on reception success
  
  DW1000.getData(rx_packet, DW1000.getDataLength());
  //Serial.println("Received something...");
  received = true;
  //show_packet(rx_packet, DW1000.getDataLength());
}

void handleError() {
  error = true;
}

void handleRxTO() {
  current_state = STATE_IDLE;
  RxTimeout = true;
  #if (DEBUG_PRINT==1)
  Serial.println("Rx Timeout");
  Serial.println("State: ");
  Serial.println(current_state);
  #endif
}

uint16_t seq = 0;

// --------------- MAIN LOGGING LOOP ---------------

void loop() {
  //update DW1000
  // DW1000Ranging.loop();

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

  SendUWBProfile();
  // if (millis() - uwbTimer >= UWB_send_timer){
  //   uwbTimer = millis();
  //   SendUWBProfile();
  // }

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

void logUWBToUSB(){
  char buff[140];
  char long_buff[1400];

  byte firstPath[2];
  DW1000.readBytes(RX_TIME, RX_TIME_FP_INDEX_OFFSET, firstPath, 2);
  uint16_t firstpath = uint16_t(firstPath[1]<<8 | firstPath[0]);
  uint16_t firstPath_integer = (firstpath & 0xFFC0) >> 6;
  uint16_t firstPath_fraction = (firstpath & 0x003F);
  float RX_POWER = DW1000.getReceivePower();
  float FP_POWER = DW1000.getFirstPathPower();
  
  uint8_t myAcc[4 * NCIR_FULL + 6];
  int starttap = 0; // 720
  int endtap = 1015; // 816 
  int16_t RealData = 0;
  int16_t ImaginaryData = 0;
  
  sprintf(long_buff, "CIR %d %d ", firstPath_fraction, firstPath_integer);
  DW1000.getAccMem(myAcc, 0, endtap + 1); //myAcc will contain 16 bit real imaginary pairs

  noInterrupts();
  if(!logFile.open(logFilename, O_APPEND | O_WRITE)) {
    Serial.println("Could not create log file.  Check SD Card.");
  }
  
  for (int i = starttap ; i < endtap + 1; i++) {
    RealData = myAcc[(i * 4) + 2] << 8 | myAcc[(i * 4) + 1];
    ImaginaryData = myAcc[(i * 4) + 4] << 8 | myAcc[(i * 4) + 3];
    sprintf(buff, "[%d,%d,%d,%d],", packet_count, RealData, ImaginaryData, i + 1);
    // strcat(long_buff, buff);
    logFile.print(buff);
  }
  logFile.println();
  interrupts();
  // Serial.println(long_buff);
}

void SendUWBProfile() {
  #if (INITIATOR==0)
    if (received) {
      received = false;
      
      logUWBToUSB();
      //print_CIR();
      seq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX+1] << 8);
      Serial.print("Received: ");
      Serial.println(seq);
      receiver(60);
    }
  #endif

  #if (INITIATOR==1)
    switch(current_state) {
    
      case STATE_IDLE: {  
        received = false;
        sendComplete = false;
        //Switch to POLL state
        current_state = STATE_POLL;
        break;
      }
      
      case STATE_POLL: {
        //Send POLL here
        seq++;
        tx_poll_msg[SRC_IDX] = myDevID;
        tx_poll_msg[DST_IDX] = BROADCAST_ID;
        tx_poll_msg[SEQ_IDX] = seq & 0xFF;
        tx_poll_msg[SEQ_IDX + 1] = seq >> 8;
        Serial.println(seq);
        
        FIXED_DELAY = 3;
        
        generic_send(tx_poll_msg, sizeof(tx_poll_msg), 0, SEND_DELAY_FIXED);
        current_state = STATE_IDLE;
        
        while(!sendComplete){
        };

        //Serial.println("Send completed");
        sendComplete = false;
        break;
      }
    }
    
  #endif
}

// //This is for Anchors only
// void newBlink(DW1000Device* device) {
//   Serial.print("blink; 1 device added ! -> ");
//   Serial.print(" short:");
//   Serial.println(device->getShortAddress(), HEX);
// }

// //This is for Tags only
// void newDevice(DW1000Device* device) {
//   Serial.print("ranging init; 1 device added ! -> ");
//   Serial.print(" short:");
//   Serial.println(device->getShortAddress(), HEX);
// }

// void inactiveDevice(DW1000Device* device) {
//   Serial.print("delete inactive device: ");
//   Serial.println(device->getShortAddress(), HEX);
// }


#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__


