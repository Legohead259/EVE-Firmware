//TODO include header protections
struct TELEMETRY {
    char timestamp[20];
    bool GPSFix;
    uint8_t numSats;
    uint8_t HDOP;               //In tenths (divide by 10. when displaying)
    long latitude;              //In millionths of a degree (divide by 1000000. when displaying)
    long longitude;             //In millionths of a degree (divide by 1000000. when displaying)
    long altitude;              //In millimeters (devide by 1000. when displaying to get meters)
    long gps_speed;             //In thousandths of a knot (divide by 1000. when displaying)
    long gps_course;            //In thousandths of a degree (divide by 1000. when displaying)
    float baro_pressure;        //Pascals
    float baro_altitude;        //Meters
    float baro_temperature;     //Celsius
    uint8_t system_cal = 0;     //IMU system calibration, 0-3 with 3 being fully calibrated
    uint8_t gyro_cal = 0;       //IMU gyroscope calibration, 0-3 with 3 being fully calibrated
    uint8_t accel_cal = 0;      //IMU accelerometer calibration, 0-3 with 3 being fully calibrated
    uint8_t mag_cal = 0;        //IMU magnetometer calibration, 0-3 with 3 being fully calibrated
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
    float imu_temperature;      //°Celsius from the IMU
    float temperature;          //°Celsius (ambient) from the SHT31-D sensor
    float humdity;              //% from the SHT31-D sensor
    int packetSize;             
};