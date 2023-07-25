//! Library Initialization Goes Here!
#include <Arduino.h>

//* Thinger Library
#include <ThingerESP32.h>
#include "arduino_secrets.h"

//* TDS Library
#include "GravityTDS.h"

//* Temperature Library
#include <OneWire.h>
#include <DallasTemperature.h>

//* RTC Library
#include <SPI.h>
#include "RTClib.h"

//! Variable and PIN Initialization  Goes Here!
//* TDS PIN
#define TDS_PIN 34

//* Thinger Initialization
ThingerESP32 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

//* WiFi Initialization
const char *ssid = "Kuro";
const char *password = "kuro_1905";

//! Variable Instance Goes Here!

//* TDS Library Insatance
GravityTDS gravityTds;

//* Temperature Library Instance
OneWire oneWire(33);
DallasTemperature DS18B20_Sensor(&oneWire);

//* RTC Instance
RTC_DS1307 rtc_DS1307;

//* Global Variable TDS and Temperature
//* Initial Value for TDS and Temperature
float tdsValue = 0;
float temperatureValue = 0;

//* Thinger Instance
bool isSendToEndpoint = false;

//! Function Declaration Goes Here!
//* FUnction declaration for readTDS
void readTDS();

//! Main Program Goes Here!
void setup()
{
  Serial.begin(115200);

  thing.add_wifi(ssid, password);

  //* Check RTC
  if (!rtc_DS1307.begin())
  {
    Serial.println("Couldn't find RTC");
    while (1)
      ;
  }

  thing["data"] >> [](pson &out)
  {
    float tdsVal;
    float tempVal;

    out["ppm"] = tdsValue;
    out["temperature"] = temperatureValue;
    out["source"] = "auto";
    out["deviceId"] = DEVICE_ID;
  };

  /* thing["ssr"] << [](pson &in)
  {
    int flow = in;
    if (globalCounter != 0)
    {
      openSelenoidValve(globalCounter, value);
    }
  }; */

  //* Gravity TDS Setup
  gravityTds.setPin(TDS_PIN);
  gravityTds.setAref(3.3);
  gravityTds.setAdcRange(4096);
  gravityTds.begin();
}

void loop()
{
  thing.handle();

  DateTime rtcCurrentTime = rtc_DS1307.now();
  readTDS();

  // You can remove isSendToEndpoint condition if not double sent like MQTT
  if (rtcCurrentTime.second() == 0 && !isSendToEndpoint)
  {
    if (rtcCurrentTime.minute() % 1 == 0)
    {
      thing.call_endpoint("TDSMeasurementsEndpoints", thing["data"]);
      Serial.println("Send to thinger");
      isSendToEndpoint = true;
    }
  }
  else if (rtcCurrentTime.second() != 0)
  {
    isSendToEndpoint = false;
  }
  // readTDS();
}
//! MAIN PROGRAM END HERE!

//! Function Definition Goes Here!
//* readTDS() function definition
void readTDS()
{
  DS18B20_Sensor.requestTemperatures();
  temperatureValue = DS18B20_Sensor.getTempCByIndex(0);
  gravityTds.setTemperature(temperatureValue);
  gravityTds.update();
  tdsValue = gravityTds.getTdsValue();

  Serial.printf("Temperature: %.2fC ", temperatureValue);
  Serial.printf("| TDS Value: %.2fppm \n", tdsValue);
}