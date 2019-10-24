#include <SPI.h>
#include <RH_RF95.h>

#define FAIL_LED 4
#define SUCCESS_LED 5

// Defaults for Arduino Uno
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 3

#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

const char SIGN[28] = "ground control to major tom";
const char COUNTERSIGN[28] = "major tom to ground control";
bool lsConnect = false;

void blink(int i=1000, int p=LED_BUILTIN);
void blinkIter(int n=1, int i=1000, int p=LED_BUILTIN);
void liten();

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(FAIL_LED, OUTPUT);
    pinMode(SUCCESS_LED, OUTPUT);
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH); //Enable RF95 radio

    Serial.begin(115200);
    while (!Serial);
    delay(100);

    Serial.println("Arduino LoRa RX Test!");
  
    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        while (1);
    }
    Serial.println("LoRa radio init OK!");

    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1);
    }
    Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

    rf95.setTxPower(23, false);

    while (!lsConnect) {
        rf95.send((uint8_t*)SIGN, 28);
        rf95.waitPacketSent();
        
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        Serial.println("Waiting for countersign..."); delay(10); //DEBUG
        if (rf95.waitAvailableTimeout(1000)) {
            if (rf95.recv(buf, &len)) {
                Serial.print("Got reply: "); Serial.println((char*)buf); //DEBUG
                // Serial.print("RSSI: "); Serial.println(rf95.lastRssi(), DEC); //DEBUG - Tells strength of last signal recieved
                if (strcmp((char*)buf, COUNTERSIGN) == 0) {
                    Serial.println("COMMUNICATION WITH LAUNCHSONDE ESTABLISHED!"); //DEBUG
                    lsConnect = true;
                    blinkIter(5, 25, SUCCESS_LED); //Visually show successful comms establishment
                    digitalWrite(SUCCESS_LED, HIGH); //Keep green high to indicate system good
                }
                else {
                    Serial.println("Wrong countersign. Are the signs right? Trying again..."); //DEBUG
                    blinkIter(3, 150, FAIL_LED); //Visually show failed comms
                }
            }
            else {
                Serial.println("Recieve failed! Trying again..."); //DEBUG
                blinkIter(3, 150, FAIL_LED); //Visually show failed comms
            }
        }
        else {
            Serial.println("No reply, is there a launchsonde around? Trying again..."); //DEBUG
            blinkIter(3, 150, FAIL_LED); //Visually show failed comms
        }
    }
}

void loop() {
    listen();
}

void listen() {
    if (rf95.waitAvailableTimeout(1000)) {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        
        if (rf95.recv(buf, &len)) {
            Serial.println((char*)buf); 
        }
        else {
            Serial.println("Receive failed");
        }
    }
}

void blink(int i=1000, int p=LED_BUILTIN) {
    digitalWrite(p, HIGH);
    delay(i);
    digitalWrite(p, LOW);
}

void blinkIter(int n=1, int i=1000, int p=LED_BUILTIN) {
    for (int x=0; x<n; x++) {
        blink(i, p);
    }
}