#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

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

// Defaults for Arduino Uno
#define RFM95_CS 10
#define RFM95_RST 3
#define RFM95_INT 2
#define RF95_FREQ 915.0
#define MY_ADDRESS 2

TELEMETRY data;
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHDatagram manager(rf95, MY_ADDRESS);

void listen();

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH); //Enable RF95 radio

    Serial.begin(115200);
    while (!Serial);
    delay(100);
  
    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    setupRadio();
}

void loop() {
    listen();
    // rf95.send((uint8_t)*"HELLOWORLD", 11); //DEBGU
    // rf95.waitPacketSent();
}

void listen() {
    if (rf95.waitAvailableTimeout(100)) {       // wait 100 mSec max for response
        uint8_t bufLen = sizeof(data);
        if (manager.recvfrom((uint8_t *) &data, &bufLen)) {    
            if (bufLen == sizeof(data)) {
                Serial.println("<======= Received timestamp = " + String(data.timestamp));
                Serial.println("<======= Received altitude = " + String(data.altitude));
                Serial.println("<======= Received sys calibration = " + String(data.system_cal));
                Serial.println("<======= Received gyro calibration = " + String(data.gyro_cal));
                Serial.println("<======= Received accel calibration = " + String(data.accel_cal));
                Serial.println("<======= Received mag calibration = " + String(data.mag_cal));
                Serial.println("<======= Received x accel = " + String(data.accelX));
                Serial.println("<======= Received y accel = " + String(data.accelY));
                Serial.println("<======= Received z accel = " + String(data.accelZ));
                Serial.println("<======= Received gyro x = " + String(data.gyroX));
                Serial.println("<======= Received gyro y = " + String(data.gyroY));
                Serial.println("<======= Received gyro z = " + String(data.gyroZ));
                Serial.println("<======= Received roll = " + String(data.roll));
                Serial.println("<======= Received pitch = " + String(data.pitch));
                Serial.println("<======= Received yaw = " + String(data.yaw));
                Serial.println("<======= Received x lin accel = " + String(data.linAccelX));
                Serial.println("<======= Received y lin accel = " + String(data.linAccelY));
                Serial.println("<======= Received z lin accel = " + String(data.linAccelZ));
            }
            else {
                Serial.println("Incorrect response size");
            }
        }
    }
}

void setupRadio(void)
{
  if (manager.init())
  {
    if (!rf95.setFrequency(RF95_FREQ))
      Serial.println("Unable to set RF95 frequency");
    // if (!rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128))
    //   Serial.println("Invalid setModemConfig() option");
    rf95.setTxPower(23);
    Serial.println("RF95 radio initialized.");
  }
  else
    Serial.println("RF95 radio initialization failed.");
    Serial.print("RF95 max message length = ");
    Serial.println(rf95.maxMessageLength());
}