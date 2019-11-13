/**
 * PROJECT PHOTON
 * FLORIDA INSTITUTE OF TECHNOLOGY, OEMS DEPARTMENT
 * UNDERWATER TECHNOLOGY LABORATORY
 * Supervising Professor: Dr. Stephen Wood, Ph.D, PE
 * 
 * Launchsonde Firmware Version 0.1.2 Created 10/21/2019 By Braidan Duffy
 * 
 * Theory of Operation:
 * This firmware is intended to contorl the lauch of the rocket through the use of an IR remote controll. Many differant remotes can be used. 
 * Firmware is currently set up to use a Onn TV remote. 
 * Upon Entering the correct password the controller will turn red signaling the rocket is armed. In this state the power button found on the TV remote 
 * will trigger the relay to lauch the rocket. The buzzer creats a tone when a IR signal is recived. The LEDs display the staus of the rocket.
 * 
 * In loop(), the firmware begins by comparing the IR string values to pre-stored hex codes, when a string is reconized the coresponding number is added 
 * to the password string. If the built password string == the set password string the rocket will be armed. If the password string is incorrect 
 * the contorller will signal wrong password throught the blueLed and buzzer tone. If the reset string is found the password string and chances is reset.
 * 
 * Last Revision: 11/13/2019 By Parker Baillon
 */
#include <IRremote.h>
#include <TimerFreeTone.h>

#define TONE_PIN 5 //Buzzer connection

//Password settings
String pass = "1234"; //Password to unlock. be sure to edit the '5' inside '[]' to password length +1.
int chances = 3; //Chances before buzzer will start to buzz continuously.
bool afterChances = true; //Ture = allow attempts after failure || False = only three attemps allowed


//Pins
int RECV_PIN = 7; //IR Reciever pin (PWM)
int greenLed = 9; //Green LED pin
int redLed = 10; //Red Led pin
int blueLed = 8; //Blue Led pin
int buzzer = 5; //Buzzer
int trigger = 6; //relay coil

//remote buttons programmable dependant on the remote choosen || Remap using the basic IR reader
String btn1 = "6bc6597b"; //Button 1 IR HEX code (in lower case). eg.: ff63ad
String btn2 = "735b797e"; 
String btn3 = "1ec81dbf";
String btn4 = "450753d6";
String btn5 = "ba0f4edf";
String btn6 = "4ac4da9a";
String btn7 = "f6317edb";
String btn8 = "f9000e7e";
String btn9 = "c7291b77";
String btn0 = "6a68351e";
String btnReset = "b8e7b4fe"; //Reset button IR hex code.

//Misc
bool shouldBeep = true;
bool unlockSound = true;

//Set up arduino
bool locked = true;
String cPass = "";
int chance = 0;
String cmp = "----Distro Studios-----\n";
IRrecv irrecv(RECV_PIN);
decode_results results;
String str2 = "os-----\n";

//Declare pinModes
void setup() {
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(blueLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(trigger, OUTPUT);
  irrecv.enableIRIn();
  Serial.begin(9600);
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, LOW);
  Serial.print(cmp);
}

//Begin Main loop to decode the string from the IR sensor and asign it a button value
void loop() {
  if (cmp.endsWith(str2) == false)
    Serial.print("Error!");
  else {
    if (irrecv.decode(&results)) {

      if (String(results.value, HEX) == btn1) {
        Serial.print("1");
        updatePass("1");
      }
      else if (String(results.value, HEX) == btn2) {
        Serial.print("2");
        updatePass("2");
      }
      else if (String(results.value, HEX) == btn3) {
        Serial.print("3");
        updatePass("3");
      }
      else if (String(results.value, HEX) == btn4) {
        Serial.print("4");
        updatePass("4");
      }
      else if (String(results.value, HEX) == btn5) {
        Serial.print("5");
        updatePass("5");
      }
      else if (String(results.value, HEX) == btn6) {
        Serial.print("6");
        updatePass("6");
      }
      else if (String(results.value, HEX) == btn7) {
        Serial.print("7");
        updatePass("7");
      }
      else if (String(results.value, HEX) == btn8) {
        Serial.print("8");
        updatePass("8");
      }
      else if (String(results.value, HEX) == btn9) {
        Serial.print("9");
        updatePass("9");
      }
      //Button to launch rocket 
      else if (String(results.value, HEX) == btn0) {
        Serial.print("0");
        updatePass("0");
        if(pass == cPass){
          Serial.println("FIRE");
          digitalWrite(greenLed, HIGH);
          digitalWrite(trigger, HIGH);
          TimerFreeTone(5,2500, 500); // Play this note for duration.
          delay(500);
          digitalWrite(trigger, LOW);
          }
          else{
          Serial.println("COLD"); 
          }
      }
      else if (String(results.value, HEX) == btnReset) {
        resetPass();
        Serial.println("\nRESET");
      }
      irrecv.resume(); // Receive the next value
    }
    if (chance > chances)
      digitalWrite(blueLed, HIGH);//buzzer to signal if all password chances have been used

    if (locked == true) { //signal rocket is safe
      digitalWrite(redLed, HIGH);
      digitalWrite(greenLed, LOW);
    }
    else { //signal rocket is armed
      digitalWrite(redLed, LOW);
      digitalWrite(greenLed, HIGH);
    }
  }
}

//Edit password length settings
bool updatePass(String ch) {
  if (locked == false)
    return false;
  beep();
  if (updatable() == true) {
    cPass += ch;
    if (cPass.length() < pass.length()) {

    }
    else {
      if (pass == cPass) {
        locked = false;
        digitalWrite(greenLed, HIGH);
        digitalWrite(redLed, LOW);
        chance = 0;
        Serial.println("\nUNLOCKED");
        if (unlockSound == true) {
          delay(60);
          digitalWrite(blueLed, HIGH);
          delay(150);
          digitalWrite(blueLed, LOW);
          delay(100);
          digitalWrite(blueLed, HIGH);
          delay(200);
          digitalWrite(blueLed, LOW);
        }
      }
      //Incorrect password tone
      else {
        cPass = "";
        chance += 1;
        digitalWrite(blueLed, HIGH);
        delay(1000);
        digitalWrite(blueLed, LOW);
        Serial.println("\nWRONG PASSWORD!");
        TimerFreeTone(5,2000, 100); // Play thisNote for duration.
        delay(100);
        TimerFreeTone(5,2500, 100); // Play thisNote for duration.
        delay(100);
        TimerFreeTone(5,2500, 100); // Play thisNote for duration.
        delay(100);
        TimerFreeTone(5,2500, 100); // Play thisNote for duration.
        delay(100);
        TimerFreeTone(5,2500, 100); // Play thisNote for duration.
        delay(100);
      }
    }
  }
}

bool updatable() {
  if (chance <= chances)
    return true;
  return afterChances;
}

void resetPass() {
  cPass = "";
  locked = true;
}

//Indication command 
void beep() {
  if (shouldBeep == true) {
    digitalWrite(blueLed, HIGH);
    TimerFreeTone(5,2500, 100); // Play thisNote for duration.
    delay(100);
    digitalWrite(blueLed, LOW);
  }
}
