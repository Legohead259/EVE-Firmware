//
// Last update: 03/25/18
//
/****************************************************/
/*  Includes                                        */
/****************************************************/
#include <RHReliableDatagram.h>
#include <RH_RF95.h>

/****************************************************/
/*  Defines                                         */
/****************************************************/
#define MY_ADDRESS      3
#define SERVER_ADDRESS  2
#define TXPWR           5
#define RFM95_CS        8
#define RFM95_INT       7

/****************************************************/
/*  Packet structure                                */
/****************************************************/
typedef struct TELEMETRY {
  char timestamp[13];
  bool GPSFix;
  float latitude;
  char lat[2];
  float longitude;
  char lon[2];
  float altitude;
};

/****************************************************/
/*  Globals                                         */
/****************************************************/
TELEMETRY data;
const float FREQ = 915.0;
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHDatagram manager(rf95, MY_ADDRESS);
// ApplicationWatchdog wd(60000, System.reset);            // Watchdog timer, will reset system after 60 seconds of inactivity

/****************************************************/
/*  setup                                           */
/****************************************************/
void setup(void) 
{
  Serial.begin(115200);
  while(!Serial); //Wait for terminal to open
  setupRadio();
}

/****************************************************/
/*  loop                                            */
/****************************************************/
void loop(void)
{
  unsigned long startMilli,
                stopMilli;
               
  strcpy(data.timestamp, "00:00:00.000");
  data.GPSFix = false;
  data.latitude = 2803.9112;
  strcpy(data.lat, "N");
  data.longitude = 8037.3013;
  strcpy(data.lon, "W");
  data.altitude = random(0, 100);

  Serial.println("=======> Sending timestamp = " + String(data.timestamp));
  Serial.println("=======> Sending gps fix = " + String(data.GPSFix));
  Serial.println("=======> Sending latitude = " + String(data.latitude, 4) + String(data.lat));
  Serial.println("=======> Sending longitude = " + String(data.longitude, 4) + String(data.lon));
  Serial.println("=======> Sending altitude = " + String(data.altitude, 2));

  startMilli = millis();
  if (!manager.sendto((uint8_t *) &data, sizeof(data), SERVER_ADDRESS))
    Serial.print("Transmit failed");
  rf95.waitPacketSent(100);                  // wait 100 mSec max for packet to be sent
  stopMilli = millis();
  Serial.print("Transmission time (mSec) = ");
  Serial.println(stopMilli-startMilli);
  Serial.println();

//   startMilli = millis();
//   if (rf95.waitAvailableTimeout(100))        // wait 100 mSec max for response
//   { 
//     uint8_t bufLen = sizeof(data);
//     if (manager.recvfrom((uint8_t *) &data, &bufLen))
//     {
//       stopMilli = millis();
//       Serial.print("Response time (mSec) = ");
//       Serial.println(stopMilli-startMilli);      
//       if (bufLen == sizeof(data))
//       {
//         Serial.println("<======= Received temp = " + String(data.temp,1));
//         Serial.println("<======= Received hum = " + String(data.hum,1));
//         Serial.println("<======= Received wind speed = " + String(data.windSpeed,0));
//         Serial.println("<======= Received wind direction = " + String(data.windDir));
//       }
//       else
//         Serial.println("Incorrect response size");
//     }
//   }
//   else
//     Serial.println("Timed out waiting for response");
//   Serial.println();
  delay(1000);
}

/****************************************************/
/*  setupRadio                                      */
/****************************************************/
void setupRadio(void)
{
  if (manager.init())
  {
    if (!rf95.setFrequency(FREQ))
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