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
                Serial.print(data.timestamp); Serial.print(", ");
                Serial.print(data.altitude); Serial.print(", ");
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
                Serial.println("");
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