/**
 * PROJECT PHOTON
 * FLORIDA INSTITUTE OF TECHNOLOGY, OEMS DEPARTMENT
 * UNDERWATER TECHNOLOGY LABORATORY
 * Supervising Professor: Dr. Stephen Wood, Ph.D, PE
 * 
 * Launchsonde Firmware Version 0.1.2 Created 10/21/2019 By Braidan Duffy
 * 
 * Theory of Operation:
 * This firmware is intended to relay instrumentation data from the launchsonde to a ground station using the LoRa RFM95 chipset
 * On startup, the firmware initializes the chipsets and prepares the file format for the SD Card logging
 * To create a file for logging, the firmware captures the current date from the onboard RTC and uses that as the name for the .txt file
 * If there has already been a file for that date created, the appropriate log number is appended to the name
 * The firmware then opens the file and if it cannot, it blocks the code from continuing further
 * NOTE: This block is non-resettable besides a full hardware reset
 * 
 * In loop(), the firmware begins by instantiating the barometer/altimeter
 * NOTE: due to a bug in the current barometer library (as of 10/21/2019), the barometer MUST be instantiated in the loop or the bootloader corrupts
 * The firmware then calibrates the barometer's starting atitude using the library's function and begins formatting the data packet
 * The data packet is formed by capturing the time from the onboard RTC and the change-in-alitude (delta-tude) from the barometer
 * The data is then put into a comma-delimitted format for data packet transmission and logging on the SD card
 * 
 * Last Revision: 10/24/2019 By Braidan Duffy
 */

#include "barometer.h"
#include "i2c.h"
#include <RH_RF95.h>
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define RFM95_FREQ 915.0

#define BNO055_SAMPLERATE_DELAY_MS 100

RH_RF95 rf95(RFM95_CS, RFM95_INT);

RTC_PCF8523 rtc;
const int chipSelect = 10;
File dataFile;
char filename[30];
bool calibrated = false;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

void recordTimestamp(char* timeStamp);
void log(char* filename, char msg);
void logTimeStamp(char* filename, char msg);

//=====ARDUINO SETUP=====
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Serial.println("Starting...");
    while(!Serial); //Wait for serial terminal to open
                    //REMOVE BEFORE FLIGHT
    Serial.println("Ending...");
    // delay(5000);s

    //----------------------------
    //---SD CARD INITIALIZATION---
    //----------------------------

    //See if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present"); //DEBUG
        while (1); //do nothing else!
    }
    Serial.println("Card initialized."); //DEBUG

    //Create the SD card file according to RTC data
    DateTime now = rtc.now();
    for (int x=0; x<100; x++) {
        sprintf(filename, "%02d%02d_%02d.txt", now.month(), now.day(), x); //ATM the filename can only be 8 chars long
        if (!SD.exists(filename)) {
            break;
        }
        // Serial.println(filename); //DEBUG
    }

    //Check if the file was successfully created
    dataFile = SD.open(filename, FILE_WRITE);
    if(!dataFile) {
        Serial.print("Couldnt create ");
        Serial.println(filename);
        while(1);
    }
    dataFile.println("TESTING LOGGER"); //DEBUG
    dataFile.close();
    Serial.print("Writing to "); Serial.println(filename); //DEBUG

    //--------------------------
    //---RFM95 INITIALIZATION---
    //--------------------------
    
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    //Maunual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!rf95.init()) { //Make sure radio initializes
        Serial.println("LoRa radio init failed!"); //DEBUG
        while (1); //do nothing else!
    }
    Serial.println("LoRa radio init OK!"); //DEBUG

    if (!rf95.setFrequency(RFM95_FREQ)) {
        Serial.println("setFrequency failed"); //DEBUG
        while (1);
    }
    Serial.print("Set Freq to: "); Serial.println(RFM95_FREQ); //DEBUG

    rf95.setTxPower(23, false); //Sets max transmitter power. Range is 5-23 dbm; default is 13 dbm

    //------------------------
    //---IMU INITIALIZATION---
    //------------------------

    //Initialize IMU
    if(!bno.begin()) {
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"); //DEBUG
        while(1);
    }

    //Calibrate IMU
    Serial.println("CALIBRATING!..."); //DEBUG
    rf95.send((uint8_t*)"CALIBRATING!...", 16);
    rf95.waitPacketSent();
    while(!calibrated) {
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        char calibrationPkt[35];
        sprintf(calibrationPkt, "Sys:%u, Gyro:%u, Accel:%u, Mag:%u", system, gyro, accel, mag); //Format calibration packet
        rf95.send((uint8_t*)calibrationPkt, sizeof(&calibrationPkt)+1);

        if (system == 3 && gyro == 3 && accel == 3 && mag == 3) { //Check if all sensors are calibrated
            calibrated = true;
            // char timeStamp[20];
            // recordTimestamp(*timeStamp);
            // dataFile = SD.open(filename, FILE_WRITE);
            // dataFile.print(timeStamp); dataFile.println("[NOTIFICATION] BNO055 calibrated!");
            // dataFile.close();
            logTimeStamp(filename, "[NOTIFICATION] BNO055 calibrated!");
            rf95.send((uint8_t*)"CALIBRATED!", 12);
            rf95.waitPacketSent();
        }

        //Calibration DEBUG
        Serial.print("CALIBRATION: Sys=");
        Serial.print(system, DEC);
        Serial.print(" Gyro=");
        Serial.print(gyro, DEC);
        Serial.print(" Accel=");
        Serial.print(accel, DEC);
        Serial.print(" Mag=");
        Serial.println(mag, DEC);
    }
}

void loop() {
    //Object instantiation placed down here to prevent bootloader corruption issues on Adafruit RFM95X 32u4 Feather
    I2C i2c;
    Barometer baro(i2c);
    baro.calibrateStartingHeight();
    float altitude = 0;

    while (1) {
        //-------------------------
        //---Timestamp Recording---
        //------------------------- 

        char timeStamp[20];
        // DateTime now = rtc.now();
        // sprintf(timeStamp, "%02d:%02d:%02d,",  now.hour(), now.minute(), now.second());
        // Serial.println(timeStamp); //DEBUG
        recordTimestamp(*timeStamp);

        //-----------------------
        //---Altimeter polling---
        //-----------------------

        altitude = baro.getAltitude();
        // Serial.print("Delta-tude: "); Serial.print(altitude); Serial.println(" m"); //DEBUG
        char altitudePkt[6+1];
        dtostrf(altitude, 6, 2, altitudePkt);

        // //-----------------
        // //---IMU polling---
        // //-----------------

        // // Possible vector values can be:
        // // - VECTOR_ACCELEROMETER - m/s^2
        // // - VECTOR_MAGNETOMETER  - uT
        // // - VECTOR_GYROSCOPE     - rad/s
        // // - VECTOR_EULER         - degrees
        // // - VECTOR_LINEARACCEL   - m/s^2
        // // - VECTOR_GRAVITY       - m/s^2
        // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        // char imuPkt[20];
        // sprintf(imuPkt, "%02d,%02d,%02d,", euler.x(), euler.y(), euler.z());
        // Serial.print("X: "); Serial.print(euler.x());   //DEBUG
        // Serial.print(" Y: "); Serial.print(euler.y());  //DEBUG
        // Serial.print(" Z: "); Serial.print(euler.z());  //DEBUG
        // Serial.println();                               //DEBUG

        //----------------------
        //---Packet formation---
        //----------------------

        char packet[sizeof(altitudePkt)+sizeof(timeStamp)] = "";
        strcat(packet, timeStamp);      //Add the timestamp into the packet
        strcat(packet, altitudePkt);    //Add the altitude data
        // strcat(packet, imuPkt);         //Add the IMU data
        // Serial.print("Sent: "); Serial.println(packet); //DEBUG
        rf95.send((uint8_t*)packet, 20); //Send data to LoRa module
        rf95.waitPacketSent(); //Wait for packet to complete
        log(filename, packet); //Log packet
    }
}

void recordTimestamp(char* timeStamp) {
    DateTime now = rtc.now();
    sprintf(timeStamp, "%02d:%02d:%02d,",  now.hour(), now.minute(), now.second());
    Serial.print("TIMESTAMP: "); Serial.println(timeStamp); //DEBUG
}

void log(char* filename, char msg) {
    dataFile = SD.open(filename, FILE_WRITE);
    dataFile.println(msg);
    Serial.print("LOG MESSAGE: "); Serial.println(msg);
    dataFile.close();
}

void logTimeStamp(char* filename, char msg) {
    char logMsg[sizeof(msg)+20];
    recordTimestamp(logMsg);
    strcat(logMsg, msg);
    log(filename, logMsg);
}