#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

typedef struct TELEMETRY {
  char timestamp[13];
  bool GPSFix;
  float latitude;
  char lat;
  float longitude;
  char lon;
  float altitude;
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
                Serial.println("<======= Received fix = " + String(data.GPSFix));
                Serial.println("<======= Received latitude = " + String(data.latitude, 4) + data.lat);
                Serial.println("<======= Received longitude = " + String(data.longitude, 4) + data.lon);
                Serial.println("<======= Received altitude = " + String(data.altitude, 2));
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