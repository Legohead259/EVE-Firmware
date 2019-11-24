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
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define RFM95_FREQ 915.0
#define MY_ADDRESS 3
#define SERVER_ADDRESS 2

#define BNO055_SAMPLERATE_DELAY_MS (100)

typedef struct TELEMETRY {
  char timestamp[20];
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

RTC_PCF8523 rtc;
const int chipSelect = 10;
File dataFile;
char filename[30];

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

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
    DateTime now = rtc.now();
    for (int x=0; x<100; x++) {
        sprintf(filename, "%02d%02d_%02d.txt", now.month(), now.day(), x); //ATM the filename can only be 8 chars long
        if (!SD.exists(filename)) {
            break;
        }
        // Serial.println(filename); //DEBUG
    }

    //Check if file can be created
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

        //---------------------
        //---Parse Timestamp---
        //---------------------

        DateTime now = rtc.now();
        uint8_t curSecond = now.second();

        if (curSecond == lastSecond) {
            curMillis = millis();
            curMSecond = curMillis - lastMillis;
            // Serial.println((int) curMSecond); //DEBUG
        }
        else {
            lastSecond = curSecond;
            lastMillis = millis();
        }

        sprintf(data.timestamp, "%02d:%02d:%02d.%d",  now.hour(), now.minute(), now.second(), (int) curMSecond);
        // Serial.println("Time: " + String(data.timestamp)); //DEBUG

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
        rf95.waitPacketSent(100); // wait 100 mSec max for packet to be sent
    }
}

boolean logTelemPacket() {
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) { //If datafile is open, write comma-delimitted telemetry packet to it.
        // dataFile.println((uint8_t *) &data);
        dataFile.print(data.timestamp); dataFile.print(", ");
        dataFile.print(data.altitude); dataFile.print(", ");
        dataFile.print(data.system_cal); dataFile.print(", ");
        dataFile.print(data.gyro_cal); dataFile.print(", ");
        dataFile.print(data.accel_cal); dataFile.print(", ");
        dataFile.print(data.mag_cal); dataFile.print(", ");
        dataFile.print(data.accelX); dataFile.print(", ");
        dataFile.print(data.accelY); dataFile.print(", ");
        dataFile.print(data.accelZ); dataFile.print(", ");
        dataFile.print(data.gyroX); dataFile.print(", ");
        dataFile.print(data.gyroY); dataFile.print(", ");
        dataFile.print(data.gyroZ); dataFile.print(", ");
        dataFile.print(data.roll); dataFile.print(", ");
        dataFile.print(data.pitch); dataFile.print(", ");
        dataFile.print(data.yaw); dataFile.print(", ");
        dataFile.print(data.linAccelX); dataFile.print(", ");
        dataFile.print(data.linAccelY); dataFile.print(", ");
        dataFile.print(data.linAccelZ); //dataFile.print(", ");
        dataFile.println("");
        dataFile.close();
        return true;
    }
    dataFile.close();
    return false;
}