/**
 * PROJECT PHOTON
 * FLORIDA INSTITUTE OF TECHNOLOGY, OEMS DEPARTMENT
 * UNDERWATER TECHNOLOGY LABORATORY
 * Supervising Professor: Dr. Stephen Wood, Ph.D, PE
 * 
 * Launchsonde Firmware Version 0.1.2 Created 10/21/2019 By Braidan Duffy
 * 
 * Theory of Operation:
 * This firmware is intended to relay instrumentation data from the launchsonde to a ground station using the LoRa RFM95 chipset
 * On startup, the firmware initializes the chipsets and prepares the file format for the SD Card logging
 * To create a file for logging, the firmware captures the current date from the onboard RTC and uses that as the name for the .txt file
 * If there has already been a file for that date created, the appropriate log number is appended to the name
 * The firmware then opens the file and if it cannot, it blocks the code from continuing further
 * Note: This block is non-resettable besides a full hardware reset
 * 
 * In loop(), the firmware begins by instantiating the barometer/altimeter
 * Note: due to a bug in the current barometer library (as of 10/21/2019), the barometer MUST be instantiated in the loop or the bootloader corrupts
 * The firmware then calibrates the barometer's starting atitude using the library's function and begins formatting the data packet
 * The data packet is formed by capturing the time from the onboard RTC and the change-in-alitude (delta-tude) from the barometer
 * The data is then put into a comma-delimitted format for data packet transmission and logging on the SD card
 * 
 * Last Revision: 10/21/2019 By Braidan Duffy
 */
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