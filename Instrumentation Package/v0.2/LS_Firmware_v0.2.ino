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
#include <SPI.h>
#include <SD.h>
#include <MicroNMEA.h>
#include <telemetry.h>

#define RFM95_CS A5
#define RFM95_RST A4
#define RFM95_INT 7
#define RFM95_FREQ 915.0
#define MY_ADDRESS 3
#define SERVER_ADDRESS 2

#define INTERRUPT_PIN 11 //PPS interrupt on board

#define BNO055_SAMPLERATE_DELAY_MS (100)

TELEMETRY data;
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHDatagram manager(rf95, MY_ADDRESS);

HardwareSerial& gps = Serial1;
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
bool ledState = LOW;
volatile bool ppsTriggered = false;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

I2C i2c;
Barometer baro(i2c);

//DEBUG Flags
const bool GPS_ECHO = false;
const bool GPS_DEBUG = false;
const bool IMU_DEBUG = false;
const bool PACKET_DEBUG = true;

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

    gps.begin(9600);

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, ledState);

	nmea.setUnknownSentenceHandler(printUnknownSentence);

	pinMode(A0, OUTPUT);
	digitalWrite(A0, HIGH);
	Serial.println("Resetting GPS module ...");
	gpsHardwareReset();
	Serial.println("... done");

	// Clear the list of messages which are sent.
	MicroNMEA::sendSentence(gps, "$PORZB");

	// Send only RMC and GGA messages.
	MicroNMEA::sendSentence(gps, "$PORZB,RMC,1,GGA,1");

	// Disable compatability mode (NV08C-CSM proprietary message) and
	// adjust precision of time and position fields
	MicroNMEA::sendSentence(gps, "$PNVGNME,2,9,1");
	// MicroNMEA::sendSentence(gps, "$PONME,2,4,1,0");

	pinMode(INTERRUPT_PIN, INPUT);
	attachInterrupt(INTERRUPT_PIN, ppsHandler, RISING);

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

    //------------------------------
    //---ALTIMETER INITIALIZATION---
    //------------------------------

    // baro.calibrateStartingHeight();
    // Serial.println("Height Calibrated!"); //DEBUG
}

void loop() {
    //-----------------
    //---GPS polling---
    //-----------------

    if (ppsTriggered) {
        ppsTriggered = false;
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);

        //TODO: Parse timestamp

        data.GPSFix = nmea.isValid();
        data.numSats = nmea.getNumSatellites();
        data.HDOP = nmea.getHDOP();
        data.latitude = nmea.getLatitude();
        data.longitude = nmea.getLongitude();
        nmea.getAltitude(data.gps_altitude);
        data.gps_speed = nmea.getSpeed();
        data.gps_course = nmea.getCourse();

        if (GPS_DEBUG) printGPSData(); //DEBUG

        nmea.clear();
    }

    while (!ppsTriggered && gps.available()) {
        char c = gps.read();
        if (GPS_ECHO) Serial.print(c);
        nmea.process(c);
    }

    //-----------------------
    //---Altimeter polling---
    //-----------------------

    // data.baro_altitudse = baro.getAltitude();
    // Serial.print("Altitude: "); Serial.print(data.altitude); Serial.println(" m"); //DEBUG

    //-----------------
    //---IMU POLLING---
    //-----------------

    //Get calibration status for each sensor.
    bno.getCalibration(&data.system_cal, &data.gyro_cal, &data.accel_cal, &data.mag_cal);

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

        //Add gyroscope data to data packet
        data.gyroX = gyro.x();
        data.gyroY = gyro.y();
        data.gyroZ = gyro.z();
        
        //Add euler rotation data to data packet
        data.roll = euler.z();
        data.pitch = euler.y();
        data.yaw = euler.x();

        //Add linear accleration data to data packet
        data.linAccelX = linaccel.x();
        data.linAccelY = linaccel.y();
        data.linAccelZ = linaccel.z();
    }
    if (IMU_DEBUG) printIMUData();

    //-------------------------
    //---Packet transmission---
    //-------------------------

    if (!manager.sendto((uint8_t *) &data, sizeof(data), SERVER_ADDRESS))
        Serial.print("Transmit failed");
    if (PACKET_DEBUG) printDataPacket();
    // rf95.waitPacketSent(100); // wait 100 mSec max for packet to be sent
}

void ppsHandler(void) {
	ppsTriggered = true;
	// Serial.println("triggered!"); //DEBUG
}

void printUnknownSentence(const MicroNMEA& nmea) {
	Serial.println();
	Serial.print("Unknown sentence: ");
	Serial.println(nmea.getSentence());
}

void gpsHardwareReset() {
	// Empty input buffer
	while (gps.available())
		gps.read();

	digitalWrite(A0, LOW);
	delay(50);
	digitalWrite(A0, HIGH);

	// Reset is complete when the first valid message is received
	while (1) {
		while (gps.available()) {
			char c = gps.read();
			if (nmea.process(c))
				return;
		}
	}
}

void printGPSData() {
    Serial.print("Date/time: ");
    Serial.print(nmea.getYear());
    Serial.print('-');
    Serial.print(int(nmea.getMonth()));
    Serial.print('-');
    Serial.print(int(nmea.getDay()));
    Serial.print('T');
    Serial.print(int(nmea.getHour()));
    Serial.print(':');
    Serial.print(int(nmea.getMinute()));
    Serial.print(':');
    Serial.print(int(nmea.getSecond()));
    Serial.print(':');
    Serial.println(int(nmea.getHundredths()));

    Serial.print("Valid fix: "); //DEBUG
    Serial.println(data.GPSFix ? "yes" : "no"); //DEBUG

    Serial.print("Num. satellites: "); //DEBUG
    Serial.println(data.numSats); //DEBUG

    Serial.print("HDOP: "); //DEBUG
    Serial.println(data.HDOP/10., 1); //DEBUG

    Serial.print("Latitude (deg): "); //DEBUG
    Serial.println(data.latitude / 1000000., 6); //DEBUG
    Serial.print("Longitude (deg): "); //DEBUG
    Serial.println(data.longitude / 1000000., 6); //DEBUG

    Serial.print("Altitude (m): ");  //DEBUG
    if (nmea.getAltitude(data.gps_altitude))
        Serial.println(data.gps_altitude / 1000., 3); //DEBUG
    else
        Serial.println("not available"); //DEBUG

    Serial.print("Speed (kts): "); //DEBUG
    Serial.println(data.gps_speed / 1000., 3); //DEBUG

    Serial.print("Course (deg): "); //DEBUG
    Serial.println(data.gps_course / 1000., 3); //DEBUG

    Serial.println("-----------------------"); //DEBUG
}

void printIMUData() {
    // Display calibration data
    Serial.print("CALIBRATION: Sys="); Serial.print(data.system_cal, DEC); //DEBUG
    Serial.print(" Gyro="); Serial.print(data.gyro_cal, DEC); //DEBUG
    Serial.print(" Accel="); Serial.print(data.accel_cal, DEC); //DEBUG
    Serial.print(" Mag="); Serial.println(data.mag_cal, DEC); //DEBUG

    // Display accelerometer data
    Serial.print("aX: "); Serial.print(data.accelX);
    Serial.print(" aY: "); Serial.print(data.accelY);
    Serial.print(" aZ: "); Serial.print(data.accelZ);
    Serial.print("\t\t");

    // Display gyroscope data
    Serial.print("gX: "); Serial.print(data.gyroX);
    Serial.print(" gY: "); Serial.print(data.gyroY);
    Serial.print(" gZ: "); Serial.print(data.gyroZ);
    Serial.print("\t\t");

    // Display euler degress data
    Serial.print("eX: "); Serial.print(data.roll);
    Serial.print(" eY: "); Serial.print(data.pitch);
    Serial.print(" eZ: "); Serial.print(data.yaw);
    Serial.print("\t\t");

    //Display linear accleration data
    Serial.print("lX: "); Serial.print(data.linAccelX);
    Serial.print(" lY: "); Serial.print(data.linAccelX);
    Serial.print(" lZ: "); Serial.print(data.linAccelX);
    Serial.print("\t\t");
    Serial.println("");
}

void printDataPacket() {
    Serial.print(data.timestamp); Serial.print(", ");
    Serial.print(data.GPSFix); Serial.print(", ");
    Serial.print(data.numSats); Serial.print(", ");
    Serial.print(data.HDOP); Serial.print(", ");
    Serial.print(data.latitude); Serial.print(", ");
    Serial.print(data.longitude); Serial.print(", ");
    Serial.print(data.gps_altitude); Serial.print(", ");
    Serial.print(data.gps_speed); Serial.print(", ");
    Serial.print(data.gps_course); Serial.print(", ");
    Serial.print(data.baro_altitude); Serial.print(", ");
    Serial.print(data.system_cal); Serial.print(", ");
    Serial.print(data.gyro_cal); Serial.print(", ");
    Serial.print(data.accel_cal); Serial.print(", ");
    Serial.print(data.mag_cal); Serial.print(", ");
    Serial.print(data.accelX); Serial.print(", ");
    Serial.print(data.accelY); Serial.print(", ");
    Serial.print(data.accelZ); Serial.print(", ");
    Serial.print(data.gyroX); Serial.print(", ");
    Serial.print(data.gyroY); Serial.print(", ");
    Serial.print(data.gyroZ); Serial.print(", ");
    Serial.print(data.roll); Serial.print(", ");
    Serial.print(data.pitch); Serial.print(", ");
    Serial.print(data.yaw); Serial.print(", ");
    Serial.print(data.linAccelX); Serial.print(", ");
    Serial.print(data.linAccelY); Serial.print(", ");
    Serial.print(data.linAccelZ); //Serial.print(", ");
    Serial.println();
}