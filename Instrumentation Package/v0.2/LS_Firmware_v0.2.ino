/**
 * PROJECT PHOTON
 * FLORIDA INSTITUTE OF TECHNOLOGY, OEMS DEPARTMENT
 * UNDERWATER TECHNOLOGY LABORATORY
 * Supervising Professor: Dr. Stephen Wood, Ph.D, PE
 * 
 * Launchsonde Firmware Version 0.1.3 Created 11/23/2019 By Braidan Duffy
 * NOTE: THERE IS A BUG THAT IS CORRUPTING THE BOOTLOADER OF THE ADAFRUIT FEATHER 32u4!!!!!!
 * 
 * 
 * Theory of Operation:
 * This firmware is intended to relay instrumentation data from the launchsonde to a ground station using the LoRa RFM95 chipset
 * On startup, the firmware initializes the chipsets and prepares the file format for the SD Card logging
 * To create a file for logging, the firmware captures the current date from the onboard RTC and uses that as the name for the .txt file
 * If there has already been a file for that date created, the appropriate log number is appended to the name
 * The firmware then opens the file and if it cannot, it blocks the code from continuing further
 * NOTE: This block is non-resettable besides a full hardware reset
 * 
 * 
 * In loop(), the firmware begins by instantiating the barometer/altimeter
 * NOTE: due to a bug in the current barometer library (as of 10/21/2019), the barometer MUST be instantiated in the loop or the bootloader corrupts
 * The firmware then calibrates the starting height of the barometer using its built-in function
 * Then, the timestamp is captured using the onboard RTC and millis() function of the microcontroller and added to the TELEMETRY data packet
 * The altitude is polled from the barometer and added to the TELEMETRY packet
 * The firmware polls the IMU and checks the calibration data from its three sensors
 * Each sensor has a calibration value from 0-3 where 0 is uncalibrated and 3 is fully calibrated
 * The firmware does not poll IMU sensor data unless the three sensors are calibrated and thus, the IMU data in the TELEMETRY packet will be 0
 * Once calibrated, the firmware polls the IMU data and adds it to the TELEMETRY packet
 * Then, the entire packet is logged to the local SD card in a comma-delimitted format by calling logTelemPacket()
 * Afterwards, the firmware sends the TELEMETRY packet to the radio datagram manager to be broadcasted to the ground station listening for data
 * This process then repeats until the unit is powered off
 * NOTE: A bug in the previous launch firmware (v0.1) stopped program execution if the SD card is ejected
 *       In this version, the instrumentation package will still transmit even without an SD card
 * 
 * Last Revision: 11/20/2019 By Braidan Duffy
 */

#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "barometer.h"
#include "i2c.h"
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GPS.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define RFM95_FREQ 915.0
#define MY_ADDRESS 3
#define SERVER_ADDRESS 2

#define BNO055_SAMPLERATE_DELAY_MS (100)

#define GPSSerial Serial1
#define GPSECHO true

typedef struct TELEMETRY {
    char timestamp[20];
    bool GPSFix;
    float latitude;
    char lat;
    float longitude;
    char lon;
    float altitude;
    uint8_t system_cal = 0;
    uint8_t gyro_cal = 0;
    uint8_t accel_cal = 0;
    uint8_t mag_cal = 0;
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float roll;
    float pitch;
    float yaw;
    float linAccelX;
    float linAccelY;
    float linAccelZ;
};

TELEMETRY data;
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHDatagram manager(rf95, MY_ADDRESS);

const int chipSelect = 10;
File dataFile;
char filename[30];

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

Adafruit_GPS GPS(&GPSSerial);
uint32_t timer = millis();

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    // while(!Serial); //Wait for serial terminal to open
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

    char c = GPS.read();

    if (GPSECHO)
        if (c) Serial.print(c); //DEBUG
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) //Parse NMEA data sentence
        return; // we can fail to parse a sentence in which case we should just wait for another
    }

    //----------------------------
    //---SD CARD INITIALIZATION---
    //----------------------------

    Serial.print("Initializing SD card...");

    //See if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        while (1); //Block code
    }
    Serial.println("card initialized.");

    //Create the file name
    for (int x=0; x<100; x++) {
        sprintf(filename, "%u%u_%i.txt", GPS.month, GPS.day, x); //ATM the filename can only be 8 chars long
        if (!SD.exists(filename)) {
            break;
        }
        // Serial.println(filename); //DEBUG
    }

    //Create the file
    dataFile = SD.open(filename, FILE_WRITE);
    if(!dataFile) {
        Serial.print("Couldnt create "); Serial.println(filename); //DEBUG
        while(1); //Block code
    }
    Serial.print("Writing to "); Serial.println(filename); //DEBUG
    dataFile.close();

    //------------------------
    //---IMU INITIALIZATION---
    //------------------------

    //Initialize sensor
    if (!bno.begin()) {
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1); //Block code
    }
    delay(1000);
    bno.setExtCrystalUse(true);
}

void loop() {
    //Object instantiation placed down here to prevent bootloader corruption issues on Adafruit RFM95X 32u4 Feather
    I2C i2c; //NOTE: Check for char something[x]; in this code, that may be that cause of corruption...
    Barometer baro(i2c); //NOTE: Check for char something[x]; in this code, that may be that cause of corruption...
    baro.calibrateStartingHeight();
    // Serial.println("Height Calibrated!"); //DEBUG

    uint8_t lastSecond = 0;
    float lastMillis = millis();
    float curMillis = 0;
    float curMSecond = 0;

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

            sprintf(data.timestamp, "%u:%u:%u:%u,", GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds); //Generate GPS timestamp string
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

        data.altitude = baro.getAltitude();
        // Serial.print("Altitude: "); Serial.print(data.altitude); Serial.println(" m"); //DEBUG

        //-----------------
        //---IMU POLLING---
        //-----------------

        //Get calibration status for each sensor.
        bno.getCalibration(&data.system_cal, &data.gyro_cal, &data.accel_cal, &data.mag_cal);
        // Serial.print("CALIBRATION: Sys="); Serial.print(data.system_cal, DEC); //DEBUG
        // Serial.print(" Gyro="); Serial.print(data.gyro_cal, DEC); //DEBUG
        // Serial.print(" Accel="); Serial.print(data.accel_cal, DEC); //DEBUG
        // Serial.print(" Mag="); Serial.println(data.mag_cal, DEC); //DEBUG

        if (data.gyro_cal == 3 && data.accel_cal == 3 && data.mag_cal == 3) {
            // - VECTOR_ACCELEROMETER - m/s^2
            // - VECTOR_GYROSCOPE     - rad/s
            // - VECTOR_EULER         - degrees
            // - VECTOR_LINEARACCEL   - m/s^2
            imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
            imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
            imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
            imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

            //Add accelerometer data to data packet            
            data.accelX = accel.x();
            data.accelY = accel.y();
            data.accelZ = accel.z();
            //Display accelerometer data DEBUG
            // Serial.print("aX: "); Serial.print(data.accelX);
            // Serial.print(" aY: "); Serial.print(data.accelY);
            // Serial.print(" aZ: "); Serial.print(data.accelZ);
            // Serial.print("\t\t");

            //Add gyroscope data to data packet
            data.gyroX = gyro.x();
            data.gyroY = gyro.y();
            data.gyroZ = gyro.z();
            //Display gyroscope data DEBUG
            // Serial.print("gX: "); Serial.print(data.gyroX);
            // Serial.print(" gY: "); Serial.print(data.gyroY);
            // Serial.print(" gZ: "); Serial.print(data.gyroZ);
            // Serial.print("\t\t");
            
            //Add euler rotation data to data packet
            data.roll = euler.z();
            data.pitch = euler.y();
            data.yaw = euler.x();
            //Display euler degress data DEBUG
            // Serial.print("eX: "); Serial.print(data.roll);
            // Serial.print(" eY: "); Serial.print(data.pitch);
            // Serial.print(" eZ: "); Serial.print(data.yaw);
            // Serial.print("\t\t");

            //Add linear accleration data to data packet
            data.linAccelX = linaccel.x();
            data.linAccelY = linaccel.y();
            data.linAccelZ = linaccel.z();
            //Display linear accleration data DEBUG
            // Serial.print("lX: "); Serial.print(data.linAccelX);
            // Serial.print(" lY: "); Serial.print(data.linAccelX);
            // Serial.print(" lZ: "); Serial.print(data.linAccelX);
            // Serial.print("\t\t");
            // Serial.println("");
        }

        //-------------------------
        //---Packet transmission---
        //-------------------------
        logTelemPacket();
        if (!manager.sendto((uint8_t *) &data, sizeof(data), SERVER_ADDRESS))
            Serial.print("Transmit failed");
        // rf95.waitPacketSent(100); // wait 100 mSec max for packet to be sent
    }
}

boolean logTelemPacket() {
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) { //If datafile is open, write comma-delimitted telemetry packet to it.
        // dataFile.println((uint8_t *) &data);
        dataFile.print(data.timestamp);         dataFile.print(", ");
        dataFile.print(data.GPSFix);            dataFile.print(", ");
        dataFile.print(data.latitude);          dataFile.print(data.lat); dataFile.print(", ");
        dataFile.print(data.longitude);         dataFile.print(data.lon); dataFile.print(", ");
        dataFile.print(data.altitude);          dataFile.print(", ");
        dataFile.print(data.system_cal);        dataFile.print(", ");
        dataFile.print(data.gyro_cal);          dataFile.print(", ");
        dataFile.print(data.accel_cal);         dataFile.print(", ");
        dataFile.print(data.mag_cal);           dataFile.print(", ");
        dataFile.print(data.accelX);            dataFile.print(", ");
        dataFile.print(data.accelY);            dataFile.print(", ");
        dataFile.print(data.accelZ);            dataFile.print(", ");
        dataFile.print(data.gyroX);             dataFile.print(", ");
        dataFile.print(data.gyroY);             dataFile.print(", ");
        dataFile.print(data.gyroZ);             dataFile.print(", ");
        dataFile.print(data.roll);              dataFile.print(", ");
        dataFile.print(data.pitch);             dataFile.print(", ");
        dataFile.print(data.yaw);               dataFile.print(", ");
        dataFile.print(data.linAccelX);         dataFile.print(", ");
        dataFile.print(data.linAccelY);         dataFile.print(", ");
        dataFile.print(data.linAccelZ);         //dataFile.print(", ");
        dataFile.println("");
        dataFile.close();
        return true;
    }
    dataFile.close();
    return false;
}