#include <MicroNMEA.h>

// To display free memory include the MemoryFree library, see
// https://github.com/maniacbug/MemoryFree and uncomment the line
// below
//#include <MemoryFree.h>

// Refer to serial devices by use
// HardwareSerial& console = Serial;
HardwareSerial& gps = Serial1;

#define INTERRUPT_PIN 11 //PPS interrupt on board

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
bool ledState = LOW;
volatile bool ppsTriggered = false;

void ppsHandler(void);

void ppsHandler(void)
{
	ppsTriggered = true;
	// Serial.println("triggered!"); //DEBUG
}

void printUnknownSentence(const MicroNMEA& nmea)
{
	Serial.println();
	Serial.print("Unknown sentence: ");
	Serial.println(nmea.getSentence());
}

void gpsHardwareReset()
{
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

void setup(void)
{
	Serial.begin(115200); // console
	gps.begin(9600); // gps

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

// #ifdef ARDUINO_AVR_CALUNIUM
	pinMode(INTERRUPT_PIN, INPUT);
	attachInterrupt(INTERRUPT_PIN, ppsHandler, RISING);
// #else
// #error Please configure interrupt handler code for alternative board.
// #endif
}

void loop(void)
{
	if (ppsTriggered) {
		ppsTriggered = false;
		ledState = !ledState;
		digitalWrite(LED_BUILTIN, ledState);

		// Output GPS information from previous second
		Serial.print("Valid fix: ");
		Serial.println(nmea.isValid() ? "yes" : "no");

		Serial.print("Nav. system: ");
		if (nmea.getNavSystem())
			Serial.println(nmea.getNavSystem());
		else
			Serial.println("none");

		Serial.print("Num. satellites: ");
		Serial.println(nmea.getNumSatellites());

		Serial.print("HDOP: ");
		Serial.println(nmea.getHDOP()/10., 1);

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

		long latitude_mdeg = nmea.getLatitude();
		long longitude_mdeg = nmea.getLongitude();
		Serial.print("Latitude (deg): ");
		Serial.println(latitude_mdeg / 1000000., 6);

		Serial.print("Longitude (deg): ");
		Serial.println(longitude_mdeg / 1000000., 6);

		long alt;
		Serial.print("Altitude (m): ");
		if (nmea.getAltitude(alt))
			Serial.println(alt / 1000., 3);
		else
			Serial.println("not available");

		Serial.print("Speed: ");
		Serial.println(nmea.getSpeed() / 1000., 3);
		Serial.print("Course: ");
		Serial.println(nmea.getCourse() / 1000., 3);
		Serial.println("-----------------------");
		nmea.clear();
	}

	while (!ppsTriggered && gps.available()) {
		char c = gps.read();
		Serial.print(c);
		nmea.process(c);
	}

}
