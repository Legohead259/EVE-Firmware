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

#include <Adafruit_MPL3115A2.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <Adafruit_GPS.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define RFM95_FREQ 915.0
#define MY_ADDRESS 3
#define SERVER_ADDRESS 2

typedef struct TELEMETRY {
  char timestamp[13];
  bool GPSFix;
  float latitude;
  char lat;
  float longitude;
  char lon;
  float altitude;
};

#define GPSSerial Serial1
#define GPSECHO true

TELEMETRY data;
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHDatagram manager(rf95, MY_ADDRESS);

Adafruit_GPS GPS(&GPSSerial);
uint32_t timer = millis();

//=====ARDUINO SETUP=====
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while(!Serial); //Wait for serial terminal to open
                    //REMOVE BEFORE FLIGHT

    //--------------------------
    //---RADIO INITIALIZATION---
    //--------------------------
    
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    //Maunual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    if (manager.init()) {
        if (!rf95.setFrequency(RFM95_FREQ))
            Serial.println("Unable to set RF95 frequency"); //DEBUG
        rf95.setTxPower(5);
        Serial.println("RF95 radio initialized."); //DEBUG
    }
    else
        Serial.println("RF95 radio initialization failed."); //DEBUG

    //------------------------
    //---GPS INITIALIZATION---
    //------------------------
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 5 Hz update rate
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ); // 5 Hz fix rate
    GPS.sendCommand(PGCMD_ANTENNA);
    delay(1000);
}

void loop() {
    //Object instantiation placed down here to prevent bootloader corruption issues on Adafruit RFM95X 32u4 Feather
    // I2C i2c; //NOTE: Check for char something[x]; in this code, that may be that cause of corruption...
    // Barometer baro(i2c); //NOTE: Check for char something[x]; in this code, that may be that cause of corruption...
    // baro.calibrateStartingHeight();
    // Serial.println("Height Calibrated!"); //DEBUG

    while (1) {

        //-----------------
        //---GPS polling---
        //-----------------
        // GPS.wakeup();
        // delay(100);
        char c = GPS.read();

        if (GPSECHO) 
            if (c) Serial.print(c); //DEBUG
        // if a sentence is received, we can check the checksum, parse it...
        if (GPS.newNMEAreceived()) {
            if (!GPS.parse(GPS.lastNMEA())) //Parse NMEA data sentence
            return; // we can fail to parse a sentence in which case we should just wait for another
        }
        // reset timer in case millis wraps around (overflow)
        if (timer > millis()) timer = millis();

        if (millis() - timer > 1150) { //Poll every 0.250 seconds
            timer = millis(); // reset the timer

            //---GETTING TIMESTAMP---
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

            sprintf(data.timestamp, "%u:%u:%u.%u,", GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds); //Generate GPS timestamp string
            Serial.println(GPS.seconds); //DEBUG
            Serial.print("Added time: "); Serial.println(data.timestamp); //DEBUG

            //---GETTING GPS DATA
            data.GPSFix = GPS.fix;
            Serial.print("Fix: "); Serial.print(data.GPSFix); //DEBUG
            // Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
            // strcat(GPSPkt, GPS.fix?1:0); //Add GPS fix boolean to GPS packet
            // Serial.print("Added fix: "); Serial.println(GPSPkt); //DEBUG

            if (GPS.fix) {
                Serial.print("Location: "); //DEBUG
                data.latitude = GPS.latitude; data.lat = GPS.lat;
                Serial.print(data.latitude, 4); Serial.print(data.lat); Serial.print(", "); //DEBUG
                data.longitude = GPS.longitude; data.lon = GPS.lon;
                Serial.print(data.longitude, 4); Serial.println(data.lon); //DEBUG
            }
            // Serial.print("Entire GPS packet: "); Serial.println(GPSPkt); //DEBUG
        }

        //-----------------------
        //---Altimeter polling---
        //-----------------------

        // data.altitude = baro.getAltitude();
        // Serial.print("Altitude: "); Serial.print(data.altitude); Serial.println(" m"); //DEBUG

        //-------------------------
        //---Packet transmission---
        //-------------------------

        //CAUSING GPS DATA TO NOT PARSE! --INTERRUPT IS CAUSING NMEA READING TO BE INTERRUPTED!
        // if (GPS.standby()) {
            if (!manager.sendto((uint8_t *) &data, sizeof(data), SERVER_ADDRESS))
                Serial.print("Transmit failed");
            rf95.waitPacketSent(100); // wait 100 mSec max for packet to be sent
        // }
    }
}