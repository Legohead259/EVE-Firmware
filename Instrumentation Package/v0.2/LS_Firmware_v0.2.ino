/**
 * PROJECT EVE
 * FLORIDA INSTITUTE OF TECHNOLOGY, OEMS DEPARTMENT
 * UNDERWATER TECHNOLOGY LABORATORY
 * Supervising Professor: Dr. Stephen Wood, Ph.D, PE
 * 
 * @file Launchsonde Firmware Version 0.2 
 * @date 12/12/2019
 * @author Braidan Duffy
 * 
 * Theory of Operation:
 * This firmware records data from onboard sensors and formats them into a data packet for transmission to a receiving ground station.
 * Currently, the implemented hardware is an RFM95W radio, MTK3339 GPS module, MPL3115A2 altimeter/barometer, BNO055 9DOF IMU, and SHT31-D humidity/temperature sensor
 * The firmware starts by instantiating and initializing each component in the order listed above
 * The radio is setup with a datagram packet manager from the RadioHead library that allows for better management of the packet transmission
 * If the radio cannot be initialized, the code is blocked from further execution
 * The GPS initializes by first opening the serial communication port and resetting it. If the serial port cannot be opened, code execution is blocked
 * Then, the firmware sends commands that control the output NMEA strings sent by the module and attaches an interrupt
 * This interrupt is connected to the PPS pin of the GPS which pulses everytime the GPS outputs a new NMEA string
 * The interrupt triggers a flag to flip, kicking off the NMEA parsing library.
 * Next, the firmware initializes the barometric altimeter
 * If the altimeter cannot be started, further code execution is blocked
 * The altimeter is then configured with the maximum oversampling ratio (see MP3115A2 datasheet for details) and activated
 * Then, the IMU is initialized and configured according to the Adafruit example code. If the sensor cannot be found, code execution is blocked
 * Finally, the humidity sensor is initialized and like the other two, code execution is blocked if it cannot be found.
 * 
 * TODO: Raise and relay errors for sensor initialization failures
 * NOTE: Execution blocks are non-resettable besides a full hardware reset
 * 
 * In loop(), each component is polled for its relevant data
 * It starts with the GPS, parsing a timestamp from the NMEA string and, if the PPS flag has been flipped, parsing proper GPS data from the NMEA string
 * Then it moves to the altimeter where barometric pressure (in Pa), altitude (m AGL), and component temperature (°C) are polled and put into the packet
 * The firmware then polls the IMU where the sensor calibration statuses are checked first. Actual data is only polled if all sensor calibrations are 3
 * Finally, the humidity sensor is polled and ambient humidity (in %) and ambient temperature (°C) are added to the telemetry packet
 * 
 * Last Revision: 12/17/2019 By Braidan Duffy
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
#include <Wire.h>
#include <MPL3115A2.h>
#include <Adafruit_SHT31.h>

#define RFM95_CS A5
#define RFM95_RST A4
#define RFM95_INT 7
#define RFM95_FREQ 915.0
#define MY_ADDRESS 3
#define SERVER_ADDRESS 2

#define INTERRUPT_PIN 11 //PPS interrupt on board

#define BNO055_SAMPLERATE_DELAY_MS (100)

#define FLIGHT_SAFE false

//==============================
//=====OBJECT INSTANTIATION=====
//==============================

//Radio and packet
TELEMETRY data;
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHDatagram manager(rf95, MY_ADDRESS);

//GPS
HardwareSerial& gps = Serial1;
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
bool ledState = LOW;
volatile bool ppsTriggered = false;
uint8_t lastSecond = 0;
float lastMillis = millis();
float curMillis = 0;
float curMSecond = 0;

//IMU
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

//Barometer
MPL3115A2 baro;

//SHT31-D
Adafruit_SHT31 sht31 = Adafruit_SHT31();

//DEBUG Flags
const bool GPS_ECHO = false;
const bool GPS_DEBUG = false;
const bool BARO_DEBUG = false;
const bool IMU_DEBUG = false;
const bool HUMID_DEBUG = false;
const bool PACKET_DEBUG = true;

//===========================
//=====ARDUINO FUNCTIONS=====
//===========================

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    if (!FLIGHT_SAFE) while(!Serial); //If code is not flight safe (i.e. in testing), wait for serial terminal to open

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
    if (!gps) {
        Serial.print("GPS Initialization failed... Check module and wiring...");
        while(1); // Blocks code
    }

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

    //------------------------------
    //---ALTIMETER INITIALIZATION---
    //------------------------------

    if (!baro.begin()) {
        Serial.println("Couldnt find sensor");
        while(1); // Block code
    }
    baro.setOversampleRate(7);
    baro.setModeActive();
    Serial.print("Calibrating starting height...");
    while(!baro.calibrateStartingHeight()); //Wait until starting height is properly calibrateds
    Serial.println("...done");

    //------------------------
    //---IMU INITIALIZATION---
    //------------------------

    if (!bno.begin()) {
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1); //Block code
    }
    delay(1000);
    bno.setExtCrystalUse(true);

    //-----------------------------
    //---HUMIDITY INITIALIZATION---
    //-----------------------------

    if (!sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
        Serial.println("Couldn't find SHT31");
        while (1); // Block code
    }
}

void loop() {
    //-----------------
    //---GPS POLLING---
    //-----------------

    //Parse milliseconds for timestamp
    uint8_t curSecond = nmea.getSecond();
    // Serial.println(curSecond);
    if (curSecond == lastSecond) {
        curMillis = millis();
        curMSecond = curMillis - lastMillis;
        // Serial.println((int) curMSecond); //DEBUG
    }
    else {
        lastSecond = curSecond;
        lastMillis = millis();
        curMSecond = 0;
    }

    //Parse timestamp
    sprintf(data.timestamp, "%02d:%02d:%02d:%03d", nmea.getHour(), nmea.getMinute(), nmea.getSecond(), (int) curMSecond);

    //Update NMEA string based on PPS pulse from GPS. By default refresh rate is 1Hz
    if (ppsTriggered) {
        ppsTriggered = false;
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);

        data.GPSFix = nmea.isValid();
        data.numSats = nmea.getNumSatellites();
        data.HDOP = nmea.getHDOP();
        data.latitude = nmea.getLatitude();
        data.longitude = nmea.getLongitude();
        nmea.getAltitude(data.altitude);
        data.gps_speed = nmea.getSpeed();
        data.gps_course = nmea.getCourse();

        if (GPS_DEBUG) printGPSData(); //DEBUG

        // nmea.clear();
    }

    while (!ppsTriggered && gps.available()) {
        char c = gps.read();
        if (GPS_ECHO) Serial.print(c);
        nmea.process(c);
    }

    //-----------------------
    //---ALTIMETER POLLING---
    //-----------------------

    baro.setModeBarometer(); //Measure pressure
    data.baro_pressure = baro.readPressure();
    baro.setModeAltimeter(); //Measure altitude
    data.baro_altitude = baro.readAltitudeAGL();
    data.baro_temperature = baro.readTemp();

    if (BARO_DEBUG) {
        Serial.print("Pressure: "); Serial.print(data.baro_pressure); Serial.println(" Pa");
        Serial.print("Altitude: "); Serial.print(data.baro_altitude); Serial.println(" m");
        Serial.print("Temperature: "); Serial.print(data.baro_temperature); Serial.println("°C");
    }

    //-----------------
    //---IMU POLLING---
    //-----------------

    //Get calibration status for each sensor.
    bno.getCalibration(&data.system_cal, &data.gyro_cal, &data.accel_cal, &data.mag_cal);

    if (data.gyro_cal == 3 && data.accel_cal == 3 && data.mag_cal == 3) {
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);    // - m/s^2
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);         // - rad/s
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);            // - degrees
        imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);   // - m/s^2

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

    data.imu_temperature = bno.getTemp();
    if (IMU_DEBUG) printIMUData();

    //----------------------
    //---HUMIDITY POLLING---
    //----------------------

    //TODO: Incorporate humidity sensor
    data.temperature = sht31.readTemperature();
    data.humdity = sht31.readHumidity();
    if (HUMID_DEBUG) {
        Serial.print("Temperature: "); Serial.print(data.temperature); Serial.println("°C");
        Serial.print("Humidity: "); Serial.print(data.humdity); Serial.println("%");
    }

    //-------------------------
    //---PACKET TRANSMISSION---
    //-------------------------

    data.packetSize = sizeof(data);
    if (!manager.sendto((uint8_t *) &data, data.packetSize, SERVER_ADDRESS))
        Serial.print("Transmit failed");
    if (PACKET_DEBUG) printDataPacket();
    // rf95.waitPacketSent(100); // wait 100 mSec max for packet to be sent
}

//===============================
//=====GPS HANDLER FUNCTIONS=====
//===============================

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

//===============================
//=====DEBUG PRINT FUNCTIONS=====
//===============================

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
    if (nmea.getAltitude(data.altitude))
        Serial.println(data.altitude / 1000., 3); //DEBUG
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

    // Display euler degrees data
    Serial.print("eX: "); Serial.print(data.roll);
    Serial.print(" eY: "); Serial.print(data.pitch);
    Serial.print(" eZ: "); Serial.print(data.yaw);
    Serial.print("\t\t");

    //Display linear accleration data
    Serial.print("lX: "); Serial.print(data.linAccelX);
    Serial.print(" lY: "); Serial.print(data.linAccelX);
    Serial.print(" lZ: "); Serial.print(data.linAccelX);
    Serial.print("\t\t");

    Serial.print("Temp: "); Serial.print(data.imu_temperature);
    Serial.println("");
}

void printDataPacket() {
    Serial.print(data.timestamp); Serial.print(", ");
    Serial.print(data.GPSFix); Serial.print(", ");
    Serial.print(data.numSats); Serial.print(", ");
    Serial.print(data.HDOP); Serial.print(", ");
    Serial.print(data.latitude); Serial.print(", ");
    Serial.print(data.longitude); Serial.print(", ");
    Serial.print(data.altitude); Serial.print(", ");
    Serial.print(data.gps_speed); Serial.print(", ");
    Serial.print(data.gps_course); Serial.print(", ");
    Serial.print(data.baro_pressure); Serial.print(", ");
    Serial.print(data.baro_altitude); Serial.print(", ");
    Serial.print(data.baro_temperature); Serial.print(", ");
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
    Serial.print(data.linAccelZ); Serial.print(", ");
    Serial.print(data.temperature); Serial.print(", ");
    Serial.print(data.humdity); Serial.print(", ");
    Serial.print(data.packetSize);
    Serial.println();
}