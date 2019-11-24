#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) {
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    /* Display the floating point data */
    Serial.print("aX: "); Serial.print(accel.x());
    Serial.print(" aY: "); Serial.print(accel.y());
    Serial.print(" aZ: "); Serial.print(accel.z());
    Serial.print("\t\t");

    /* Display the floating point data */
    Serial.print("gX: "); Serial.print(gyro.x());
    Serial.print(" gY: "); Serial.print(gyro.y());
    Serial.print(" gZ: "); Serial.print(gyro.z());
    Serial.print("\t\t");
    
    /* Display the floating point data */
    Serial.print("eX: "); Serial.print(euler.x());
    Serial.print(" eY: "); Serial.print(euler.y());
    Serial.print(" eZ: "); Serial.print(euler.z());
    Serial.print("\t\t");

    /* Display the floating point data */
    Serial.print("lX: "); Serial.print(linaccel.x());
    Serial.print(" lY: "); Serial.print(linaccel.y());
    Serial.print(" lZ: "); Serial.print(linaccel.z());
    Serial.print("\t\t");

    /*
    // Quaternion data
    imu::Quaternion quat = bno.getQuat();
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.print("\t\t");
    */

    /* Display calibration status for each sensor. */
    uint8_t system, gyro_cal, accel_cal, mag_cal = 0;
    bno.getCalibration(&system, &gyro_cal, &accel_cal, &mag_cal);
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro_cal, DEC);
    Serial.print(" Accel=");
    Serial.print(accel_cal, DEC);
    Serial.print(" Mag=");
    Serial.println(mag_cal, DEC);

    delay(BNO055_SAMPLERATE_DELAY_MS);
}
