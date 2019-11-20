/**
 * PROJECT PHOTON
 * FLORIDA INSTITUTE OF TECHNOLOGY, OEMS DEPARTMENT
 * UNDERWATER TECHNOLOGY LABORATORY
 * Supervising Professor: Dr. Stephen Wood, Ph.D, PE
 * 
 * Launchsonde Firmware Version 0.1.2 Created 10/21/2019 By Braidan Duffy
 * NOTE: THERE IS A BUG THAT IS CORRUPTING THE BOOTLOADER OF THE ADAFRUIT FEATHER 32u4!!!!!!
 * 
 * ~=~=~=~DO NOT USE!~=~=~=~
 * 
 * Theory of Operation:
 * TODO: Update
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
 * Last Revision: 11/20/2019 By Braidan Duffy
 */

#include "barometer.h"
#include "i2c.h"
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <Adafruit_GPS.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define RFM95_FREQ 915.0

#define GPSSerial Serial1
#define GPSECHO false

RH_RF95 rf95(RFM95_CS, RFM95_INT);

Adafruit_GPS GPS(&GPSSerial);
uint32_t timer = millis();

//=====ARDUINO SETUP=====
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while(!Serial); //Wait for serial terminal to open
                    //REMOVE BEFORE FLIGHT

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
    //---GPS INITIALIZATION---
    //------------------------
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ); // 5 Hz update rate
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ); // 5 Hz fix rate
    GPS.sendCommand(PGCMD_ANTENNA);
    delay(1000);
}

void loop() {
    //Object instantiation placed down here to prevent bootloader corruption issues on Adafruit RFM95X 32u4 Feather
    // I2C i2c;
    // Barometer baro(i2c);
    // baro.calibrateStartingHeight();
    // Serial.println("Height Calibrated!"); //DEBUG
    // float altitude = 0;

    while (1) {

        //-----------------
        //---GPS polling---
        //-----------------

        char GPSPkt[48] = ""; //12 for timestamp, 4 for fix, 22 for Lat/Lon coords, 7 for altitude (>1000'), 3 as overflow buffer
        char c = GPS.read();
        // if a sentence is received, we can check the checksum, parse it...
        if (GPS.newNMEAreceived()) {
            if (!GPS.parse(GPS.lastNMEA())) //Parse NMEA data sentence
            return; // we can fail to parse a sentence in which case we should just wait for another
        }
        // reset timer in case millis wraps around (overflow)
        if (timer > millis()) timer = millis();

        if (millis() - timer > 250) { //Poll every 0.250 seconds
            timer = millis(); // reset the timer

            //---GETTING TIMESTAMP---

            char timestamp[13] = ""; 
            //---DEBUG---
            Serial.print("\nTime: ");
            if (GPS.hour < 10) { Serial.print('0'); }
            Serial.print(GPS.hour); Serial.print(':');
            //Minutes
            if (GPS.minute < 10) { Serial.print('0'); }
            Serial.print(GPS.minute); Serial.print(':');
            //Seconds
            if (GPS.seconds < 10) { Serial.print('0'); }
            Serial.print(GPS.seconds, DEC); Serial.print('.');
            //Milliseconds
            if (GPS.milliseconds < 10) {
            Serial.print("00");
            } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
            Serial.print("0");
            }
            Serial.println(GPS.milliseconds);
            //------------

            sprintf(timestamp, "%u:%u:%u.%u,", GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds); //Generate GPS timestamp string
            // Serial.print("Added time: "); Serial.println(timestamp); //DEBUG
            strcat(GPSPkt, timestamp);

            Serial.print("Fix: "); Serial.print((int)GPS.fix);
            // Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
            strcat(GPSPkt, GPS.fix?1:0); //Add GPS fix boolean to GPS packet
            Serial.print("Added fix: "); Serial.println(GPSPkt); //DEBUG

            if (GPS.fix) {
                // Serial.println("GPS FIXED!");
                Serial.print("Location: ");
                Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
                Serial.print(", ");
                Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
                // Serial.print("Speed (knots): "); Serial.println(GPS.speed);
                // Serial.print("Angle: "); Serial.println(GPS.angle);
                // Serial.print("Altitude: "); Serial.println(GPS.altitude);
                // Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
            }
            Serial.print("Entire GPS packet: "); Serial.println(GPSPkt); //DEBUG
        }

        //-----------------------
        //---Altimeter polling---
        //-----------------------

        // altitude = baro.getAltitude();
        // Serial.print("Delta-tude: "); Serial.print(altitude); Serial.println(" m"); //DEBUG
        // char altitudePkt[6+1];
        // dtostrf(altitude, 6, 2, altitudePkt);

        //----------------------
        //---Packet formation---
        //----------------------

        // char packet[sizeof(altitudePkt)+sizeof(timeStamp)] = "";
        // char packet[35] = "";
        // strcat(packet, timeStamp);      //Add the timestamp into the packet
        // strcat(packet, altitudePkt);    //Add the altitude data
        // strcat(packet, imuPkt);         //Add the IMU data
        // strcat(packet, GPS)
        // Serial.print("Sent: "); Serial.println(packet); //DEBUG
        // rf95.send((uint8_t*)packet, 35); //Send data to LoRa module
        // rf95.waitPacketSent(); //Wait for packet to complete
        // log(filename, packet); //Log packet
    }
}